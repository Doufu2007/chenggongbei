"""One-source offline rectification study. No network, ROS or class truth.

Manual quadrilateral and crop candidates are analysis annotations only.
Run from any cwd; this new exploratory directory is the sole output location.
"""
from __future__ import annotations

from datetime import datetime, timezone
import gc
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageOps

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from tools.analyze_platform_camera_frames import load_local_clip
from tools.diagnose_clip_image_neighbors import contact_sheets, cosine_neighbors, digest, font, load_verified_image_encoder
from tools.evaluate_reference_vision import DEFAULT_ASSETS, load_assets, split_assets
from inspection_mission.vision_core import TemplateMatcher, detect_targets


SOURCE = ROOT/'runs/analysis/20261112_approach_v16b/native_L000020_a672e7739804717c.jpg'
SUMMARY = SOURCE.parent/'summary.json'
QUAD = np.float32([[267, 477], [524, 477], [592, 531], [198, 531]])


def square(image):
    edge = max(image.size)
    canvas = Image.new('RGB', (edge, edge), (127, 127, 127))
    canvas.paste(image, ((edge-image.width)//2, (edge-image.height)//2))
    return canvas


def main():
    cv2.setNumThreads(1)
    cv2.setRNGSeed(20261112)
    summary = json.loads(SUMMARY.read_text(encoding='utf-8'))
    identity = next(item for item in summary['native_images'] if item['file_name'] == SOURCE.name)
    if digest(SOURCE) != identity['sha256']:
        raise ValueError('real source image hash mismatch')
    with Image.open(SOURCE) as loaded:
        original = loaded.convert('RGB')
    pixels = np.array(original)
    candidates = []

    def add(name, image, description, **details):
        path = OUT/(name+'.png')
        image.save(path)
        candidates.append({'name': name, 'path': path.name, 'sha256': digest(path),
            'size_wh': list(image.size), 'description': description, **details})

    overlay = original.copy()
    draw = ImageDraw.Draw(overlay)
    draw.line([tuple(p) for p in QUAD]+[tuple(QUAD[0])], fill=(255, 0, 0), width=2)
    for index, point in enumerate(QUAD):
        draw.text(tuple(point+[3, -17]), str(index), fill=(255, 0, 0), font=font(15))
    overlay.save(OUT/'manual_quad_overlay.png')
    add('full_scene', original, 'Unmodified decoded original JPEG scene.')
    tight = original.crop((198, 477, 593, 532))
    add('oblique_bbox', tight, 'Unwarped bbox; official CLIP center crop discards both wide ends.', bbox_xyxy=[198, 477, 593, 532])
    add('oblique_bbox_letterbox', square(tight), 'Unwarped bbox with neutral padding preserving all observed content.')
    context = original.crop((140, 420, 660, 570))
    add('scene_context', context, 'Larger unchanged scene region around board; contains blue floor and other markings.', bbox_xyxy=[140, 420, 660, 570])
    add('scene_context_letterbox', square(context), 'Same scene context padded to retain its full width in CLIP.')

    def warp(name, quad, width, height, margin=0):
        pad_x, pad_y = round(width*margin), round(height*margin)
        dst = np.float32([[pad_x, pad_y], [pad_x+width-1, pad_y],
                          [pad_x+width-1, pad_y+height-1], [pad_x, pad_y+height-1]])
        transform = cv2.getPerspectiveTransform(quad, dst)
        size = width+2*pad_x, height+2*pad_y
        warped = cv2.warpPerspective(pixels, transform, size, flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(127, 127, 127))
        mask = cv2.warpPerspective(np.full(pixels.shape[:2], 255, np.uint8), transform,
            size, flags=cv2.INTER_NEAREST, borderValue=0)
        add(name, Image.fromarray(warped), 'Manual-quad projective resampling; no detail restoration.',
            manual_quad_xy=quad.astype(float).tolist(), output_board_size_wh=[width, height],
            external_context_margin_fraction=margin, homography_source_to_output=transform.tolist(),
            valid_source_fraction=float((mask>0).mean()))

    warp('warp_4_3', QUAD, 384, 288)
    warp('warp_square', QUAD, 384, 384)
    warp('warp_4_3_context', QUAD, 384, 288, .12)
    expanded = QUAD.copy(); expanded[:2, 1] -= 2; expanded[2:, 1] += 2
    contracted = QUAD.copy(); contracted[:2, 1] += 2; contracted[2:, 1] -= 2
    warp('warp_4_3_expand_y2', expanded, 384, 288)
    warp('warp_4_3_contract_y2', contracted, 384, 288)

    assets, manifest = load_assets(DEFAULT_ASSETS)
    references, held_out, excluded = split_assets(assets)
    matcher = TemplateMatcher({a['class_id']: a['absolute_path'] for a in references})
    classifier, clip_record = load_local_clip(ROOT/'external_resources/openai_CLIP')
    variants = []
    orb = cv2.ORB_create(nfeatures=2200, fastThreshold=8, edgeThreshold=15)
    for candidate in candidates:
        with Image.open(OUT/candidate['path']) as loaded:
            base = loaded.convert('RGB')
        for angle in (0, 90, 180, 270):
            variant = base.rotate(angle, expand=True)
            variant.save(OUT/f"{candidate['name']}_ccw_{angle:03d}.png")
            array = np.array(variant)
            bgr = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
            cv2.setRNGSeed(20261112)
            began = perf_counter()
            detections = detect_targets(bgr, enable_qr=False, enable_aruco=False, template_matcher=matcher)
            template_ms = (perf_counter()-began)*1000
            began = perf_counter(); classified = classifier(array); clip_ms = (perf_counter()-began)*1000
            variants.append({'candidate': candidate['name'], 'rotation_ccw_deg': angle,
                'clip_fixed_protocol': classified, 'template_candidates': detections,
                'raw_orb_keypoints': len(orb.detect(cv2.cvtColor(array, cv2.COLOR_RGB2GRAY), None)),
                'timing_ms': {'template': template_ms, 'clip': clip_ms}})
    del classifier
    gc.collect()
    import torch
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
    encode, image_encoder_record = load_verified_image_encoder(ROOT/'external_resources/openai_CLIP')
    query_features = []
    for candidate in candidates:
        with Image.open(OUT/candidate['path']) as loaded:
            base = loaded.convert('RGB')
        query_features.append(encode([base.rotate(angle, expand=True) for angle in (0, 90, 180, 270)]))
    image_features = []
    for start in range(0, len(assets), 4):
        images = []
        for asset in assets[start:start+4]:
            with Image.open(asset['absolute_path']) as loaded:
                images.append(loaded.convert('RGB'))
        image_features.append(encode(images))
    qf, rf = np.vstack(query_features), np.vstack(image_features)
    neighbors = cosine_neighbors(qf, rf, [Path(a['path']).name for a in assets])
    np.savez(OUT/'image_features.npz', query=qf, reference=rf)
    with Image.open(OUT/'warp_4_3.png') as loaded:
        board = loaded.convert('RGB')
    board_results = cosine_neighbors(qf[20:24], rf, [Path(a['path']).name for a in assets])
    contact_sheets([(angle, board.rotate(angle, expand=True)) for angle in (0, 90, 180, 270)],
        assets, board_results, OUT)
    for variant, ranked in zip(variants, neighbors['rankings']):
        variant['reference_neighbors'] = ranked
    gallery = Image.new('RGB', (1600, 1000), (27, 27, 27))
    draw = ImageDraw.Draw(gallery)
    draw.text((12, 10), 'One real frame / manual geometry / exploratory only / no class truth', fill='white', font=font(22))
    for i, candidate in enumerate(candidates):
        x, y = (i%5)*320, 55+(i//5)*465
        with Image.open(OUT/candidate['path']) as loaded:
            thumb = ImageOps.contain(loaded.convert('RGB'), (310, 340))
        gallery.paste(thumb, (x+(320-thumb.width)//2, y+(345-thumb.height)//2))
        result = variants[i*4]['clip_fixed_protocol']
        for line_no, text in enumerate((candidate['name'], result['closed_set_class'],
            f"cos={result['top1_cosine']:.4f} margin={result['margin']:.4f}", result['decision'])):
            draw.text((x+8, y+350+22*line_no), text, fill='white', font=font(15))
    gallery.save(OUT/'candidate_gallery.png')
    report = {'kind': 'single_real_frame_rectification_exploratory', 'created_at': datetime.now(timezone.utc).isoformat(),
        'status': 'completed', 'source_image': {'path': str(SOURCE.relative_to(ROOT)), 'sha256': digest(SOURCE),
            'size_wh': list(original.size), 'source_metadata': identity},
        'source_summary': {'path': str(SUMMARY.relative_to(ROOT)), 'sha256': digest(SUMMARY)},
        'manual_annotation': {'quad_xy': QUAD.tolist(), 'source_vertical_extent_px': 54,
            'method': 'Parent-provided approximate visually reviewed full-board corners; not runtime detection.',
            'aspect_ratio_status': 'unverified sensitivity candidates; no board ID or class used'},
        'protocol': {'candidate_selection': '10 fixed source/ROI/context/warp candidates x 4 rotations, all preserved.',
            'clip': clip_record, 'image_encoder': image_encoder_record,
            'opencv': {'threshold_changes': False, 'source_sha256': digest(ROOT/'src/inspection_mission/inspection_mission/vision_core.py'),
                'reference_paths': [a['path'] for a in references]},
            'assets': {'repository': manifest['repository'], 'commit_sha': manifest['commit_sha'],
                'manifest_sha256': digest(DEFAULT_ASSETS/'SOURCE_MANIFEST.json'), 'official_current_library_verified': False}},
        'candidates': candidates, 'variants': variants,
        'image_neighbor_cosine_matrix': neighbors['cosine_matrix'],
        'references_in_column_order': assets, 'class_ground_truth': None, 'recognized_class': None,
        'accuracy': None, 'threshold_tuning': False, 'runtime_corner_detector': False, 'deployed': False,
        'published_to_referee': False, 'official_scoring': 'unverified',
        'limitations': ['All 40 variants share one JPEG and are not independent observations.',
            '54 source pixels vertically; projective interpolation and padding create no new measured detail.',
            'Approximate corners and assumed output aspect ratios can change scores.',
            'Fixed CLIP acceptance is an uncalibrated exploratory candidate, not a truth label.',
            'All 12 annotated third-party references are neighbors, not verified current simulator textures.',
            'Source is retained diagnostic JPEG, not original raw camera bytes; no new image is acquired.']}
    (OUT/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'output': str(OUT), 'variants': len(variants),
        'clip_accepted': sum(v['clip_fixed_protocol']['decision']=='accepted' for v in variants),
        'template_nonempty': sum(bool(v['template_candidates']) for v in variants)}, ensure_ascii=True))


if __name__ == '__main__':
    main()

"""Strict read-only v33 decoder; creates results only from a real input file.

No platform calls, ROS imports, robot output or generated platform evidence.
The frozen v28 numerical helpers are reused; v33 identity is checked locally.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import zlib

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
DEFAULT_SOURCE = ROOT / "runs/evidence/20261112/callback_relocated_reverse_v33.jsonl"
MODE = "ground_relocated_reverse"
SECTION = "ground_relocated_reverse_evidence_gzip_part"
CORE = "ground_relocated_reverse_core"
SCHEMA = "ground_relocated_reverse_evidence_v1"
BUNDLE = "ground_relocated_reverse_v1"
RAW_KEYS = frozenset(("schema", "raw_odometry", "raw_scans", "commands", "state_records", "zero_requests"))
METADATA = ("mode", "section", "encoding", "session_id", "bundle",
            "uncompressed_sha256", "compressed_sha256", "uncompressed_bytes", "compressed_bytes", "part_count")
MAX_RAW, MAX_PACKED = 32 * 1024 * 1024, 8 * 1024 * 1024

SPEC = importlib.util.spec_from_file_location("frozen_v28_numerical_helpers", OUT.parent / "20261112_direction_v28/analyze.py")
api = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(api)
api.MODE = MODE
strict_json, stamp, odom, response, wrap = api.strict_json, api.stamp, api.odom, api.response, api.wrap
write_json, csv_rows = api.write_json, api.csv_rows


def decode_parts(parts, cores):
    if not parts:
        raise ValueError("v33 bundle not yet received")
    first = parts[0]
    if any(key not in first for key in METADATA):
        raise ValueError("incomplete v33 metadata")
    if (first["mode"] != MODE or first["section"] != SECTION or first["bundle"] != BUNDLE
            or first["encoding"] != "gzip+base64" or not isinstance(first["session_id"], str)
            or not 1 <= len(first["session_id"]) <= 128):
        raise ValueError("v33 identity mismatch")
    for key, cap in (("uncompressed_bytes",MAX_RAW),("compressed_bytes",MAX_PACKED),("part_count",512)):
        if type(first[key]) is not int or not 1 <= first[key] <= cap:
            raise ValueError("invalid bounded size/count: " + key)
    for key in ("uncompressed_sha256","compressed_sha256"):
        if not isinstance(first[key],str) or not re.fullmatch("[0-9a-f]{64}",first[key]):
            raise ValueError("invalid SHA256 metadata")
    chunks, duplicates = {}, 0
    for part in parts:
        if any(part.get(k) != first[k] for k in METADATA):
            raise ValueError("conflicting v33 session/metadata")
        index, encoded = part.get("part_index"), part.get("base64_part")
        if (type(index) is not int or not 0 <= index < first["part_count"]
                or not isinstance(encoded,str) or not 0 < len(encoded) <= 48000
                or not re.fullmatch("[A-Za-z0-9+/=]+",encoded)):
            raise ValueError("invalid part index/base64")
        if index in chunks:
            if chunks[index] != encoded:
                raise ValueError("conflicting duplicate part")
            duplicates += 1
        chunks[index] = encoded
    if len(chunks) != first["part_count"]:
        raise ValueError(f"incomplete v33 parts {len(chunks)}/{first['part_count']}")
    encoded = "".join(chunks[i] for i in range(first["part_count"]))
    if len(encoded) > 4*((MAX_PACKED+2)//3):
        raise ValueError("base64 exceeds bound")
    packed = base64.b64decode(encoded,validate=True)
    if len(packed) != first["compressed_bytes"] or hashlib.sha256(packed).hexdigest() != first["compressed_sha256"]:
        raise ValueError("compressed length/SHA mismatch")
    decoder = zlib.decompressobj(16+zlib.MAX_WBITS)
    raw = decoder.decompress(packed,first["uncompressed_bytes"]+1)
    if (not decoder.eof or decoder.unused_data or decoder.unconsumed_tail
            or len(raw) != first["uncompressed_bytes"]
            or hashlib.sha256(raw).hexdigest() != first["uncompressed_sha256"]):
        raise ValueError("raw stream/length/SHA mismatch")
    body = strict_json(raw.decode("utf-8"))
    if (body.get("mode") != MODE or body.get("schema") != SCHEMA
            or body.get("session_id") != first["session_id"]):
        raise ValueError("decoded v33 schema/session mismatch")
    for name in ("raw_odometry","raw_scans","commands","state_records","zero_requests","pulse_records","turn_records"):
        if not isinstance(body.get(name),list):
            raise ValueError("missing evidence list: "+name)
    if body.get("relocated_contract",{}).get("revision") != BUNDLE:
        raise ValueError("relocated contract revision mismatch")
    expected_core = {k:v for k,v in body.items() if k not in RAW_KEYS}
    if not cores:
        raise ValueError("separate v33 core not received")
    if any(not isinstance(c,dict) or c != expected_core for c in cores):
        raise ValueError("separate core full key/value set differs from bundle")
    return body, packed, raw, {**{k:first[k] for k in METADATA},"duplicate_parts":duplicates,"core_count":len(cores)}


def path_window(trajectory, start_raw, end_raw):
    a,b = odom(start_raw), odom(end_raw)
    if b["source_stamp_s"] < a["source_stamp_s"]:
        raise ValueError("window source time reversed")
    inside = [r for r in trajectory if a["source_stamp_s"] < r["source_stamp_s"] < b["source_stamp_s"]]
    samples = [a]+inside+([b] if b["source_stamp_s"] > a["source_stamp_s"] else [])
    path = sum(math.dist(x["xyz_m"],y["xyz_m"]) for x,y in zip(samples,samples[1:]))
    rotation = sum(abs(wrap(y["yaw_rad"]-x["yaw_rad"])) for x,y in zip(samples,samples[1:]))
    signed = sum(wrap(y["yaw_rad"]-x["yaw_rad"]) for x,y in zip(samples,samples[1:]))
    return {"start":a,"end":b,"source_dt_s":b["source_stamp_s"]-a["source_stamp_s"],
            "cumulative_path_m":path,"cumulative_absolute_rotation_rad":rotation,
            "signed_rotation_rad":signed,"maximum_net_distance_from_start_m":max(math.dist(a["xyz_m"],r["xyz_m"]) for r in samples),
            "response":response(start_raw,end_raw),"sample_count":len(samples)},samples


def first_last_nonzero(commands):
    nonzero = [c for c in commands if c["sent"] and (c["linear_x"] or c["linear_y"] or c["angular_z"])]
    zeros = [c for c in commands if c["sent"] and c["linear_x"] == c["linear_y"] == c["angular_z"] == 0.
             and nonzero and c["wall_elapsed_s"] >= nonzero[-1]["wall_elapsed_s"]]
    return {"nonzero_count":len(nonzero),"failed_publications":sum(not c["sent"] for c in commands),
            "first_nonzero":nonzero[0] if nonzero else None,"last_nonzero":nonzero[-1] if nonzero else None,
            "first_successful_zero_after_last_nonzero":zeros[0] if zeros else None,
            "first_nonzero_to_first_successful_zero_wall_s":None if not nonzero or not zeros else zeros[0]["wall_elapsed_s"]-nonzero[0]["wall_elapsed_s"]}


def augment(body, metrics, trajectory):
    if not trajectory:
        raise ValueError("no actual odometry available for numerical analysis")
    final_raw = body["raw_odometry"][-1]["payload"]
    commands = body["commands"]
    windows = []
    turns = []
    for record in body["turn_records"]:
        end = record.get("settled",{}).get("raw_odom",final_raw)
        measured, samples = path_window(trajectory,record["start_raw_odom"],end)
        first_wall = record["start_wall_elapsed_s"]
        last_wall = record.get("settled",{}).get("wall_elapsed_s",trajectory[-1]["wall_elapsed_s"])
        turn_commands = [c for c in commands if first_wall <= c["wall_elapsed_s"] <= last_wall
                         and c["phase"] in ("TURN","TURN_SETTLE")]
        row = {"index":record["index"],"stage_index":record["stage_index"],"target_yaw_rad":record["target_yaw_rad"],
               "settled":record.get("settled"),"reported_elapsed_s":record.get("elapsed_s"),
               "reconstructed_elapsed_s":last_wall-first_wall,"actual":measured,
               "final_target_error_rad":wrap(record["target_yaw_rad"]-measured["end"]["yaw_rad"]),
               "reported_path_origin_m":record["start_path_m"],"reported_rotation_origin_rad":record["start_rotation_rad"],
               "publications":first_last_nonzero(turn_commands)}
        turns.append(row)
        windows.append(("turn_"+str(record["index"]),record["start_raw_odom"],end,first_wall,last_wall))
    pre = body.get("preposition_record")
    preposition = None
    if pre:
        end = pre.get("settled",{}).get("raw_odom",final_raw)
        actual,samples = path_window(trajectory,pre["start_raw_odom"],end)
        start_xy = actual["start"]["xyz_m"]
        end_xyz = actual["end"]["xyz_m"]
        first_wall = pre["start_wall_elapsed_s"]
        last_wall = pre.get("settled",{}).get("wall_elapsed_s",trajectory[-1]["wall_elapsed_s"])
        cs = [c for c in commands if first_wall <= c["wall_elapsed_s"] <= last_wall
              and c["phase"] in ("PREPOSITION","PREPOSITION_SETTLE")]
        preposition = {"actual":actual,"settled":pre.get("settled"),
            "reconstructed_south_progress_m":start_xy[1]-end_xyz[1],
            "maximum_abs_world_x_cross_track_m":max(abs(r["xyz_m"][0]-start_xy[0]) for r in samples),
            "maximum_northward_backtrack_m":max(0.,max(r["xyz_m"][1]-start_xy[1] for r in samples)),
            "actual_endpoint_error_m":math.dist(end_xyz,pre["target_xyz_m"]),
            "target_xyz_m":pre["target_xyz_m"],"reported_path_origin_m":pre["start_path_m"],
            "request_x_mps":pre["requested_body_x_mps"],"publications":first_last_nonzero(cs),
            "elapsed_until_observed_end_wall_s":last_wall-first_wall}
        windows.append(("preposition",pre["start_raw_odom"],end,first_wall,last_wall))
    for record in body["pulse_records"]:
        end = record.get("settled",{}).get("raw_odom",final_raw)
        first_wall = record["start"]["wall_elapsed_s"]
        last_wall = record.get("settled",{}).get("wall_elapsed_s",trajectory[-1]["wall_elapsed_s"])
        windows.append(("pulse_"+str(record["index"]),record["start"]["raw_odom"],end,first_wall,last_wall))
    zeros = []
    for index,zero in enumerate(body["zero_requests"]):
        start = zero.get("raw_odom")
        if start is None:
            zeros.append({"index":index,"available":False,"zero_request":zero})
            continue
        matches = [w for w in windows if w[3]-1e-9 <= zero["wall_elapsed_s"] <= w[4]+1e-9]
        window = min(matches,key=lambda w:w[4]) if matches else None
        end = window[2] if window else final_raw
        cutoff_wall = window[4] if window else trajectory[-1]["wall_elapsed_s"]
        resumed_nonzero = next((c for c in commands if c["sent"] and
            (c["linear_x"] or c["linear_y"] or c["angular_z"])
            and zero["wall_elapsed_s"] < c["wall_elapsed_s"] < cutoff_wall), None)
        # A PAUSE zero request ceases to bound physical stopping once motion
        # is deliberately re-authorized. Do not count resumed travel as drift.
        if resumed_nonzero is not None:
            candidates = [(i,r) for i,r in enumerate(trajectory)
                          if stamp(start) <= r["source_stamp_s"]
                          and r["wall_elapsed_s"] <= resumed_nonzero["wall_elapsed_s"]]
            if candidates:
                i,_ = candidates[-1]
                end = body["raw_odometry"][i]["payload"]
        if stamp(end) < stamp(start):
            raise ValueError("zero-request end before its raw source pose")
        actual,_ = path_window(trajectory,start,end)
        zeros.append({"index":index,"available":True,"window":None if window is None else window[0],
                      "zero_request":zero,"actual_after_request":actual,
                      "within_10mm_observed_stop_path":actual["cumulative_path_m"] <= .01+1e-10,
                      "resumed_nonzero_cuts_observation_window":resumed_nonzero,
                      "interpretation":"pose observations after request; actuator application latency remains unobserved"})
    reconstruction = []
    start_index = metrics["reconstruction_start_index"]
    if start_index is not None:
        accumulated_path=accumulated_rotation=0.
        rows = trajectory[start_index:]
        reconstructed_by_stamp = {rows[0]["source_stamp_s"]:(0.,0.)}
        for a,b in zip(rows,rows[1:]):
            accumulated_path += math.dist(a["xyz_m"],b["xyz_m"])
            accumulated_rotation += abs(wrap(b["yaw_rad"]-a["yaw_rad"]))
            reconstructed_by_stamp[b["source_stamp_s"]] = accumulated_path,accumulated_rotation
        for label,raw,_,wall,_ in sorted(windows,key=lambda w:w[3]):
            key=stamp(raw)
            value=reconstructed_by_stamp.get(key)
            source = next((t for t in body["turn_records"] if label == "turn_"+str(t["index"])),None)
            if source:
                reported_path,reported_angle=source["start_path_m"],source["start_rotation_rad"]
            elif label == "preposition":
                reported_path,reported_angle=pre["start_path_m"],None
            else:
                pulse=next(p for p in body["pulse_records"] if label == "pulse_"+str(p["index"]))
                reported_path,reported_angle=pulse["start"]["path_origin_m"],pulse["start"]["rotation_origin_rad"]
            reconstruction.append({"window":label,"source_stamp_s":key,"start_wall_s":wall,
                "reported_path_m":reported_path,"reported_rotation_rad":reported_angle,
                "reconstructed_path_m":None if value is None else value[0],
                "reconstructed_rotation_rad":None if value is None else value[1],
                "path_origin_matches_global_ledger":None if value is None else abs(value[0]-reported_path)<=1e-10,
                "rotation_origin_matches_global_ledger":None if value is None or reported_angle is None else abs(value[1]-reported_angle)<=1e-10})
    scans = []
    for row in body["raw_scans"]:
        payload=row["payload"];source=stamp(payload)
        ranges=payload["ranges"]
        if not isinstance(ranges,list) or not ranges or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in ranges):
            raise ValueError("accepted raw scan has invalid/nonfinite ranges")
        prior = next((r for r in reversed(trajectory) if r["source_stamp_s"] <= source),None)
        beam=min(range(len(ranges)),key=ranges.__getitem__)
        scans.append({"source_stamp_s":source,"wall_elapsed_s":row["wall_elapsed_s"],"minimum_range_m":ranges[beam],
            "minimum_beam_index":beam,"minimum_body_angle_rad":payload["angle_min"]+beam*payload["angle_increment"],
            "strict_actual_prior_odom_source_s":None if prior is None else prior["source_stamp_s"],
            "strict_actual_prior_skew_s":None if prior is None else source-prior["source_stamp_s"],
            "prior_within_50ms":False if prior is None else source-prior["source_stamp_s"]<=.05+1e-9})
    rejected = body.get("rejected_scan")
    rejected_diagnosis = None
    if rejected:
        payload=rejected["payload"]
        try:
            source=stamp(payload)
            prior=next((r for r in reversed(trajectory) if r["source_stamp_s"]<=source),None)
            pairs=[(v,i) for i,v in enumerate(payload.get("ranges",[])) if isinstance(v,(float,int)) and math.isfinite(v)]
            value,beam=min(pairs) if pairs else (None,None)
            rejected_diagnosis={"reason":rejected["reason"],"source_stamp_s":source,
                "wall_elapsed_s":rejected["wall_elapsed_s"],"minimum_finite_range_m":value,"minimum_beam_index":beam,
                "body_angle_rad":None if beam is None else payload["angle_min"]+beam*payload["angle_increment"],
                "actual_strict_prior_odom":prior,
                "actual_prior_skew_s":None if prior is None else source-prior["source_stamp_s"],
                "complete_payload_preserved_in":"evidence.json/rejected_scan/payload"}
        except (KeyError,TypeError,ValueError) as exc:
            rejected_diagnosis={"reason":rejected.get("reason"),"diagnostic":"rejected malformed frame retained: "+str(exc),
                                "complete_payload_preserved_in":"evidence.json/rejected_scan/payload"}
    complete=(body["reason"]=="relocated_reverse_response_observed_official_score_unverified"
              and len(turns)==3 and all(t["settled"] for t in turns) and preposition is not None and preposition["settled"]
              and len(metrics["pulses"])==3 and all(p["settled_record_present"] for p in metrics["pulses"]))
    for pulse in metrics["pulses"]:
        actual=pulse["reconstructed_response"]
        pulse["measured_opposes_initial_body_x"]=(actual.get("available",False)
            and actual["delta_start_body_xy_m"][0]<-.0005)
    metrics.update(schema="ground_relocated_reverse_analysis_v1",turns=turns,preposition=preposition,
        stage_ledger_reconstruction=reconstruction,zero_request_all_windows=zeros,
        accepted_scan_summary=scans,rejected_scan_diagnosis=rejected_diagnosis,
        calibration_three_pulses_observed_complete=bool(complete),not_completed_reason=None if complete else body["reason"],
        all_three_measured_reverse_responses=bool(complete) and all(p["measured_opposes_initial_body_x"] for p in metrics["pulses"]),
        total_turn_elapsed_reported_s=body["total_turn_elapsed_s"],
        completed_turn_elapsed_reconstructed_s=sum(t["reconstructed_elapsed_s"] for t in turns if t["settled"]),
        drops_zero=all(value==0 for value in body["dropped"].values()),
        ledger_no_reset_supported=bool(reconstruction) and all(r["path_origin_matches_global_ledger"] is True
            and r["rotation_origin_matches_global_ledger"] in (None,True) for r in reconstruction)
            and metrics["path_matches"] is True and metrics["rotation_matches"] is True)
    return metrics,scans


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",type=Path,default=DEFAULT_SOURCE)
    parser.add_argument("--output",type=Path,default=OUT)
    parser.add_argument("--platform-id",default=None)
    parser.add_argument("--run-id",default=None)
    args=parser.parse_args()
    if not args.input.exists() or not args.input.stat().st_size:
        print("WAITING: real v33 callback file missing/empty; no results written")
        return 2
    raw_source=args.input.read_bytes()
    try:
        rows=[strict_json(line) for line in raw_source.decode("utf-8").splitlines() if line.strip()]
        data_rows=[(r,r.get("payload",r).get("data",{})) for r in rows]
        parts=[d for _,d in data_rows if d.get("section")==SECTION]
        cores=[d.get("data") for _,d in data_rows if d.get("section")==CORE]
        body,packed,raw,meta=decode_parts(parts,cores)
    except (ValueError,KeyError,TypeError,zlib.error,UnicodeError) as exc:
        print("INCOMPLETE_OR_REJECTED:",str(exc))
        return 2
    if args.input.resolve().parent==args.output.resolve():
        raise ValueError("keep callbacks separate from analysis output")
    # Preserve exact recovered bytes and validation before numerical derivation.
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/"evidence.json.gz").write_bytes(packed)
    (args.output/"evidence.json").write_bytes(raw)
    write_json(args.output/"events_without_parts.json",[r for r,d in data_rows if d.get("section")!=SECTION])
    validation={"source":str(args.input.resolve()),"source_sha256":hashlib.sha256(raw_source).hexdigest(),
        "source_bytes":len(raw_source),"event_count":len(rows),"metadata":meta,
        "parts_verified":meta["part_count"],"core_full_key_value_matches":True,
        "compressed_length_matches":True,"compressed_hash_matches":True,"raw_length_matches":True,"raw_hash_matches":True,
        "platform_id":args.platform_id,"run_id":args.run_id,"session_id":body["session_id"],
        "callback_sources":sorted(set(r.get("source","unspecified") for r in rows)),
        "last_received_at":rows[-1].get("received_at"),
        "launcher_exit_codes":[d.get("exit_code") for r,d in data_rows if r.get("payload",r).get("event")=="launcher_exit"],
        "source_authenticated_independently":False}
    write_json(args.output/"validation.json",validation)
    metrics,trajectory,fd=api.analyze(body)
    metrics,scans=augment(body,metrics,trajectory)
    metrics.update(platform_id=args.platform_id,run_id=args.run_id,input_sha256=meta["uncompressed_sha256"])
    write_json(args.output/"metrics.json",metrics)
    csv_rows(args.output/"trajectory.csv",trajectory)
    csv_rows(args.output/"commands.csv",body["commands"])
    csv_rows(args.output/"finite_differences.csv",fd)
    csv_rows(args.output/"accepted_scans.csv",scans)
    print(json.dumps({k:metrics[k] for k in ("state","reason","calibration_three_pulses_observed_complete",
        "all_three_measured_reverse_responses","pulse_count","path_matches","rotation_matches","ledger_no_reset_supported","drops_zero")},ensure_ascii=False))
    return 0


if __name__=="__main__":
    raise SystemExit(main())

from pathlib import Path
import base64,gzip,hashlib,json
source=Path('runs/evidence/20261112/callback_vertical_v27.jsonl')
out=Path('runs/analysis/20261112_vertical_v27')
out.mkdir(parents=True,exist_ok=True)
rows=[json.loads(x) for x in source.read_text('utf-8').splitlines() if x.strip()]
parts=[r['payload']['data'] for r in rows if r['payload']['data'].get('section')=='bounded_vertical_gzip_part']
assert len(parts)==4
parts.sort(key=lambda p:p['part_index'])
keys=('session_id','encoding','uncompressed_bytes','compressed_bytes','uncompressed_sha256','compressed_sha256','part_count')
assert all(tuple(p[k] for k in keys)==tuple(parts[0][k] for k in keys) for p in parts)
assert [p['part_index'] for p in parts]==list(range(parts[0]['part_count']))
encoded=''.join(p['base64_part'] for p in parts)
compressed=base64.b64decode(encoded,validate=True)
assert len(compressed)==parts[0]['compressed_bytes']
assert hashlib.sha256(compressed).hexdigest()==parts[0]['compressed_sha256']
raw=gzip.decompress(compressed)
assert len(raw)==parts[0]['uncompressed_bytes']
assert hashlib.sha256(raw).hexdigest()==parts[0]['uncompressed_sha256']
body=json.loads(raw)
assert body['session_id']==parts[0]['session_id']
core=next(r['payload']['data'] for r in rows if r['payload']['data'].get('section')=='bounded_vertical_core')
assert all(body[k]==v for k,v in core.items() if k in body)
(out/'evidence.json.gz').write_bytes(compressed)
(out/'evidence.json').write_bytes(raw)
(out/'events_without_parts.json').write_text(json.dumps([r for r in rows if r['payload']['data'].get('section')!='bounded_vertical_gzip_part'],ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
validation={'source':source.as_posix(),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'source_bytes':source.stat().st_size,'event_count':len(rows),'parts_verified':len(parts),'session_id':body['session_id'],'metadata':{k:parts[0][k] for k in keys},'compressed_length_matches':True,'compressed_hash_matches':True,'raw_length_matches':True,'raw_hash_matches':True,'core_matches_recovered':True,'dropped':body['dropped'],'sample_counts':{k:len(body[k]) for k in ('trajectory','commands','scans','downscans','tf_records')},'callback_source':'unverified_callback','platform_id':'997eb6e5-bdda-4abc-88c2-ecb252319b4a','source_authenticated_independently':False}
(out/'validation.json').write_text(json.dumps(validation,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(validation,ensure_ascii=False,indent=2))
print('BODY_KEYS',list(body))
print('FIRST_TRAJECTORY',body['trajectory'][0])
print('LAST_TRAJECTORY',body['trajectory'][-1])
print('FIRST_COMMAND',body['commands'][0])
print('LAST_COMMAND',body['commands'][-1])

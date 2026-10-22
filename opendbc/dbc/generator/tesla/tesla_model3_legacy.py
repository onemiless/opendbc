from pathlib import Path


def generate():
  content = (Path(__file__).resolve().parents[2] / "tesla_model3_party.dbc").read_text()
  content = content.replace('DAS_steeringControlType : 23|3@0+ (1,0) [0|7]',
                            'DAS_steeringControlType : 23|2@0+ (1,0) [0|3]')
  content = content.replace(' 4 "FSD"', '')
  return {"tesla_model3_party_legacy.dbc": content}

import json

def build_response(status: str, message: str = "") -> str:
    return json.dumps({"status": status, "message": message})

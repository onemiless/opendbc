import json
import os
import threading
import time


DYNAMIC_ACC_DEBUG_PATH = "/data/dynamic_acc_debug.log"
_MAX_LOG_BYTES = 2 * 1024 * 1024
_LOG_LOCK = threading.Lock()


def _append_dynamic_acc_debug(record: dict) -> None:
  try:
    with _LOG_LOCK:
      if os.path.exists(DYNAMIC_ACC_DEBUG_PATH) and os.path.getsize(DYNAMIC_ACC_DEBUG_PATH) > _MAX_LOG_BYTES:
        os.replace(DYNAMIC_ACC_DEBUG_PATH, f"{DYNAMIC_ACC_DEBUG_PATH}.1")
      with open(DYNAMIC_ACC_DEBUG_PATH, "a", encoding="utf-8") as log_file:
        log_file.write(json.dumps(record, sort_keys=True, default=str) + "\n")
  except OSError:
    pass


def log_dynamic_acc(source: str, event: str, *, sync: bool = False, **values) -> None:
  record = {
    "wall_time": time.time(),
    "monotonic_ns": time.monotonic_ns(),
    "source": source,
    "event": event,
    **values,
  }
  if sync:
    _append_dynamic_acc_debug(record)
  else:
    threading.Thread(target=_append_dynamic_acc_debug, args=(record,), daemon=True).start()

"""
Central logging configuration.

Call setup_logging() once, as early as possible in the process (main.py),
before any other robo-pi module does real work. Every module logs via the
standard `logging.getLogger(__name__)` pattern and propagates up to the
root logger — this just points the root logger's handlers at a rotating
log file plus the console, so nothing else needs to change.

Log file: logs/robo-pi.log (repo root), rotated at 5 MB, 3 backups kept.
Path is gitignored (see .gitignore: *.log) — created on first run.
"""
import logging
import logging.handlers
import pathlib

_LOG_DIR = pathlib.Path(__file__).parent.parent.parent.parent / "logs"
_LOG_FILE = _LOG_DIR / "robo-pi.log"
_MAX_BYTES = 5 * 1024 * 1024  # 5 MB per file
_BACKUP_COUNT = 3             # robo-pi.log.1 .. .3

_FORMAT = "%(asctime)s %(levelname)-8s %(name)s: %(message)s"
_DATEFMT = "%Y-%m-%d %H:%M:%S"

_configured = False


def setup_logging(level: int = logging.INFO) -> None:
    """Attach a rotating file handler + console handler to the root logger.

    Safe to call more than once — later calls are a no-op so re-imports
    (e.g. under pytest) don't stack duplicate handlers.
    """
    global _configured
    if _configured:
        return
    _configured = True

    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter(_FORMAT, datefmt=_DATEFMT)

    file_handler = logging.handlers.RotatingFileHandler(
        _LOG_FILE, maxBytes=_MAX_BYTES, backupCount=_BACKUP_COUNT
    )
    file_handler.setFormatter(formatter)

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)

    root = logging.getLogger()
    root.setLevel(level)
    root.addHandler(file_handler)
    root.addHandler(console_handler)

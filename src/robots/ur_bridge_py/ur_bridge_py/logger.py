import inspect


class Logger:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[96m'
    NC = '\033[0m'

    @classmethod
    def _get_caller_class(cls):
        """Return the caller's class name if available, else empty string."""
        frame = inspect.currentframe()
        outer_frames = inspect.getouterframes(frame)
        for record in outer_frames:
            if 'self' in record.frame.f_locals:
                return record.frame.f_locals['self'].__class__.__name__
        return ""

    @classmethod
    def _build(cls, level: str, color: str, message: str, show_class: bool = True) -> str:
        """Build formatted log string, optionally include caller class name."""
        caller_cls = cls._get_caller_class() if show_class else ""
        prefix = f"{color}[{level}]{cls.NC}"
        if caller_cls:
            return f"{prefix} ({caller_cls}) {message}"
        else:
            return f"{prefix} {message}"

    @classmethod
    def success(cls, message: str, show_class: bool = True) -> str:
        return cls._build("SUCCESS", cls.GREEN, message, show_class)

    @classmethod
    def error(cls, message: str, show_class: bool = True) -> str:
        return cls._build("ERROR", cls.RED, message, show_class)

    @classmethod
    def warning(cls, message: str, show_class: bool = True) -> str:
        return cls._build("WARNING", cls.YELLOW, message, show_class)

    @classmethod
    def info(cls, message: str, show_class: bool = True) -> str:
        return cls._build("INFO", cls.BLUE, message, show_class)

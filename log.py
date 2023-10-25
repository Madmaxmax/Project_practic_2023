import logging
from colorama import init, Fore, Style

logger = logging.getLogger(__name__)


def start_log():
    init(autoreset=True)

    log_format = '[%(asctime)s] %(levelname)s | %(name)s | %(message)s'

    error_format = logging.Formatter('[%(asctime)s] ERROR | %(name)s | ERROR | %(message)s')
    error_handler = logging.StreamHandler()
    error_handler.setFormatter(error_format)
    error_handler.setLevel(logging.ERROR)

    logging.basicConfig(format=log_format, level=logging.INFO)

    logging.addLevelName(logging.INFO, f"{Fore.CYAN}{logging.getLevelName(logging.INFO)}{Style.RESET_ALL}")
    logging.addLevelName(logging.WARNING, f"{Fore.MAGENTA}{logging.getLevelName(logging.WARNING)}{Style.RESET_ALL}")
    logging.addLevelName(logging.ERROR, f"{Fore.RED}{logging.getLevelName(logging.ERROR)}{Style.RESET_ALL}")



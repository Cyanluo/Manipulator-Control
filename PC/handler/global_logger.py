import logging
import logging.config
from os import path
log_file_path = path.join(path.dirname(path.abspath(__file__)), "../config/logger.conf")
print(log_file_path)


def get_logger(name='root'):
    logging.config.fileConfig(log_file_path)
    return logging.getLogger(name)


logger = get_logger(__name__)

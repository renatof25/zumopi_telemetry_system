#!/usr/bin/env python3

#  Telemetry Project:
#  Renato and Michael.
import datetime
import logging
from threading import Timer
from time import time


class RepeatedTimer(object):
  def __init__(self, interval, function, *args, **kwargs):
    self._timer = None
    self.interval = interval
    self.function = function
    self.args = args
    self.kwargs = kwargs
    self.is_running = False
    self.next_call = time()
    self.start()

  def _run(self):
    self.is_running = False
    self.start()
    self.function(*self.args, **self.kwargs)

  def start(self):
    if not self.is_running:
      self.next_call += self.interval
      self._timer = Timer(self.next_call - time(), self._run)
      self._timer.start()
      self.is_running = True

  def stop(self):
    self._timer.cancel()
    self.is_running = False


class TelemetryLogger():
    def __init__(self, name, path, *,console=True, disabled=False, level=logging.INFO):    
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        self.logger = logging.getLogger(name)

        file_handler = logging.FileHandler(path)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

        if console:
            stdout_handler = logging.StreamHandler()
            stdout_handler.setFormatter(formatter)
            self.logger.addHandler(stdout_handler)

        self.logger.disabled = disabled

        self.logger.setLevel(level)

def log_msg(logger, pre_name, msg):
    logger.info(f"{pre_name.title()}: {'.'.join(str(msg).split('.')[2:])}")


def generate_str_timestamp():
    now = datetime.datetime.now()
    return now.strftime("%Y-%m-%d_%H-%M-%S")

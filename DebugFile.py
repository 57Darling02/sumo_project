# DEBUG parameters
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from datetime import datetime
realrundata = []
planrundata = []
controlrundata = []
class logger:
    def __init__(self):
        self.DEBUG = DEBUG
        self.start_time = datetime.now()
        self.log_info(f"TimeStamp:{self.start_time}")
        self.log_info(f"Import debug module successful")
        return

    def log_info(self,msg):
        now = datetime.now()
        time_diff = now - self.start_time
        formatted_time = format_time_diff(time_diff).rjust(10)
        print(f'{formatted_time}   {msg}')
        return 0

    def log_debug(self,msg):
        if self.DEBUG:
            self.log_info(msg)
            return 0
        return 1
def format_time_diff(delta):
    return f"{delta.total_seconds():.3f}"
def log_info(msg):
    return Logger.log_info(msg)
def log_debug(msg):
    return Logger.log_debug(msg)

DEBUG = False
Logger = logger()
def draw_debug(hh, title=None, ax=None):
    x, y = zip(*hh)
    if ax is None:
        ax = plt.gca()  # 如果没有传入axes，就获取当前的axes
    # 在传入的axes上绘制折线图
    ax.plot(x, y, marker='o')
    ax.set_title(title)
    ax.grid(True)

if __name__ == '__main__':
    pass

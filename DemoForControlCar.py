import DebugFile
from DebugFile import log_info
import Init
import Monitor
import matplotlib.pyplot as plt
def main():
    log_info('Start sumo simulation...')
    sumo = Init.sumo_controller
    # Running the simulation loop
    while sumo.running():
        try:
            Monitor.Monitor_update()
        except Exception as e:
            log_info(f"Exception occurred: {e}")  # More detailed error logging

        sumo.step()  # Progress the simulation step
    # 主程序
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))  # 创建1行3列的子图

    # 分别绘制三张图
    DebugFile.draw_debug(DebugFile.planrundata, 'planrun', ax=axes[0])
    DebugFile.draw_debug(DebugFile.controlrundata, 'controlrun', ax=axes[1])
    DebugFile.draw_debug(DebugFile.realrundata, 'realrun', ax=axes[2])

    plt.tight_layout()  # 自动调整子图布局
    plt.show()  # 一次性显示所有图像
    sumo.close()  # Close SUMO simulation



if __name__ == '__main__':
    main()

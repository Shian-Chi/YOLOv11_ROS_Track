import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))
from utils import CSVHandler, system_time

class LogData:
    def __init__(self):
        self.csv_data = {
            "time": system_time(),
            "GPS/RTK": "None",
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude(m)": 0.0,
            "H": 0.0,
            "detectionCount": 0,
            "gimbalYawDeg(°)": 0.0,
            "gimbalPitchDeg(°)": 0.0,
            "gimbalYawMove(°)": 0.0,
            "gimbalPitchMove(°)": 0.0,
            "gimbalCenter": False, 
            "FPS": 0.0,
            "centerDistance": None,
            "Bbox_x1": 0, "Bbox_x2": 0,
            "Bbox_y1": 0, "Bbox_y2": 0,
            "distanceVisual": 0.0,
            "distanceActual": 0.0,
            "thetaDeg(°)": 0.0,
            "phiDeg(°)": 0.0
        }

class LogWrite(CSVHandler, LogData):
    def __init__(self, save_path):
        super().__init__(save_path)  # super 會依據 MRO 順序執行 CSVHandler 的 __init__
        LogData.__init__(self)
        self.logData = LogData()
        self.save_path = save_path

if __name__ == "__main__":
    import time
    print("1." ,time.time())
    log = LogWrite("/home/ubuntu/track/track2/test/log_data.csv")
    print("2." ,time.time())
    log.csv_data["FPS"] = 10
    log.write_row()
    print("3." ,time.time())
    log.csv_data["FPS"] = 20
    log.write_row()
    print("4." ,time.time())
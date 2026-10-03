from pathlib import Path
from ultralytics import YOLO

def main():
    file_path = Path(__file__).resolve().parent
    data = file_path / "Turtlebots.yolov8"
    config = file_path / "turtlebot_data.yaml"
    config.write_text(
        f"path: '{data.as_posix()}'\n"
        "train: train/images\n"
        "val: valid/images\n"
        "test: test/images\n"
        "names:\n"
        "  0: turtle\n"
    )
    model = YOLO("yolo11n.pt")
    model.train(
        data=str(config),
        epochs = 10,
        imgsz = 320,
        batch = 2,
        device = "cpu",
        workers = 0,
        project = str(file_path / "runs"),
        name = "turtlebot",
    )
    print("Training finished!")
    print("Model saved in:", model.trainer.save_dir / "weights" / "best.pt")

if __name__ == "__main__":
    main()
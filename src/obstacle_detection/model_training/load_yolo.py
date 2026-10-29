from roboflow import Roboflow
rf = Roboflow(api_key="h5oZQblzwYERJ1ExtdSa")
project = rf.workspace("ricci-rp").project("yolov11m-seg")
version = project.version(1)
dataset = version.download("coco-segmentation")
                
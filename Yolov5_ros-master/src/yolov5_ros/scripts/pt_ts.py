import torch
import torchvision

# 你模型的一个实例.
model = torch.hub.load("/home/yami/Yolov5_ros-master/src/yolov5_ros/yolov5", 'custom', path="/home/yami/Yolov5_ros-master/src/yolov5_ros/weights/weightR/best1_100.pt", source='local')
# script_module = torch.jit.script(model)
# script_module.save("model.pt")
# model = torchvision.models.resnet18()
# # 您通常会提供给模型的forward()方法的示例输入。
# example = torch.rand(1, 3, 224, 224)
# # 使用`torch.jit.trace `来通过跟踪生成`torch.jit.ScriptModule`
# traced_script_module = torch.jit.trace(model, example)

# An instan
 
# An example input you would normally provide to your model's forward() method.
example = torch.rand(1, 3, 256 , 128)
# Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
traced_script_module = torch.jit.trace(model, example)
traced_script_module.save("model.pt")
import onnx

# ONNX 모델 열기
model = onnx.load("policy.onnx")

# 모델 구조 출력
print(onnx.helper.printable_graph(model.graph))

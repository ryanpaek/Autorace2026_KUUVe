#!/usr/bin/env bash
# 11/16: YOLO ONNX -> TensorRT 엔진 (리모에서 한 번, 모델 바꿀 때마다). FP16, 몇 분 걸림.
#   bash tools/build_trt.sh [onnx 경로]   (기본: ~/autorace_ws/models/autorace_yolo_v5.onnx)
# 결과: 같은 폴더 같은 이름 .engine -> detector_node가 자동으로 GPU 사용 (없으면 CPU)
# 마지막에 trtexec가 찍는 "GPU Compute Time ... mean" 이 한 장 처리 시간
set -e
ONNX=${1:-$HOME/autorace_ws/models/autorace_yolo_v5.onnx}
ENGINE=${ONNX%.onnx}.engine
TRTEXEC=/usr/src/tensorrt/bin/trtexec
[ -f "$ONNX" ] || { echo "ONNX 없음: $ONNX"; exit 1; }
echo "만드는 중: $ENGINE (주행 노드 끈 상태에서)"
$TRTEXEC --onnx="$ONNX" --saveEngine="$ENGINE.tmp" --fp16 --memPoolSize=workspace:1024 2>&1 \
  | tee /tmp/build_trt.log | grep -E "GPU Compute Time|Throughput|ERROR|PASSED|FAILED"
mv "$ENGINE.tmp" "$ENGINE"
echo "완료: $ENGINE"

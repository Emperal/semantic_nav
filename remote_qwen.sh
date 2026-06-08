#!/bin/bash

# PASSWORD="你的密码"
# USER="roo"
# HOST="192.168.5.37"

ssh roo@192.168.5.37 << 'EOF'
# sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no $USER@$HOST << 'EOF'

# 进入conda环境
source ~/.bashrc
conda activate model_qwen_310

# CUDA环境变量
export CUDA_HOME="$CONDA_PREFIX"
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$CUDA_HOME/lib:$CUDA_HOME/lib64:$LD_LIBRARY_PATH"
export LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$CUDA_HOME/lib:$CUDA_HOME/lib64:$LIBRARY_PATH"
export LDFLAGS="-L/usr/lib/x86_64-linux-gnu $LDFLAGS"

# 清缓存
rm -rf ~/.cache/flashinfer

# 启动 vLLM
vllm serve /home/roo/.cache/modelscope/hub/models/Qwen/Qwen3.5-35B-A3B \
  --host 127.0.0.1 \
  --port 8000 \
  --served-model-name Qwen/Qwen3.5-35B-A3B \
  --reasoning-parser qwen3 \
  --default-chat-template-kwargs '{"enable_thinking": false}' \
  --max-model-len 1024 \
  --gpu-memory-utilization 0.6 \
  --max-num-seqs 2 \
  --max-num-batched-tokens 256 \
  --limit-mm-per-prompt '{"image": 1}' \
EOF

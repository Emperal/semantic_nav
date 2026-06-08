# PySLAM
## 1. create conda env
```bash
conda create -n pyslam python=3.11.9 -y
```

## 2. activate env
```bash
conda activate pyslam
```

## 3. install pytorch on jetson thor
```bash 
pip install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu130
```

## install pyqt5
```bash
conda install -c conda-forge pyqt=5 pyqtgraph -y
```

## 4. install pyslam env using scripts
```bash
cd pyslam
./install_all.sh
```
- it'll take a while, just wait until it complete
- pyQt5 will not install succesfully in `jetson thor`

## 5. 

# install bson-rpc 
```bash
python -m pip install bson-rpc --no-deps --break-system-packages
```
- this lib should install in system env directly
# 1.安装torch

先使用nvidia-smi查看可以安装cuda的最高版本

![image-20231206110344599](C:\Users\xjtu\AppData\Roaming\Typora\typora-user-images\image-20231206110344599.png)

打开pytorch官网，这里选择安装cuda11.7的pytorch，找到之前的版本的

![image-20231206110454189](C:\Users\xjtu\AppData\Roaming\Typora\typora-user-images\image-20231206110454189.png)

可以使用pip安装

![image-20231206110557295](C:\Users\xjtu\AppData\Roaming\Typora\typora-user-images\image-20231206110557295.png)

```sh
# ROCM 5.2 (Linux only)
pip install torch==1.13.0+rocm5.2 torchvision==0.14.0+rocm5.2 torchaudio==0.13.0 --extra-index-url https://download.pytorch.org/whl/rocm5.2
# CUDA 11.6
pip install torch==1.13.0+cu116 torchvision==0.14.0+cu116 torchaudio==0.13.0 --extra-index-url https://download.pytorch.org/whl/cu116
# CUDA 11.7
pip install torch==1.13.0+cu117 torchvision==0.14.0+cu117 torchaudio==0.13.0 --extra-index-url https://download.pytorch.org/whl/cu117
# CPU only
pip install torch==1.13.0+cpu torchvision==0.14.0+cpu torchaudio==0.13.0 --extra-index-url https://download.pytorch.org/whl/cpu
```

# 2.安装CUDA

搜索cuda11.7

![image-20231206110715518](C:\Users\xjtu\AppData\Roaming\Typora\typora-user-images\image-20231206110715518.png)

进入CUDA，加载的时间优点长

然后选择cuda11.7 linux 对应ubuntu版本的，根据命令行下载即可

```
https://developer.nvidia.com/cuda-11-7-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_local
```

![image-20231206110923200](C:\Users\xjtu\AppData\Roaming\Typora\typora-user-images\image-20231206110923200.png)

根据下面的命令进行安装即可

```sh
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2004-11-7-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
```



# 3.下载cudnn

搜索cudnn，然后进行下载和安装，首先要进行邮箱注册和验证

![image-20231206111246081](C:\Users\xjtu\AppData\Roaming\Typora\typora-user-images\image-20231206111246081.png)

选择这个画红线的安装

![image-20231206111423048](C:\Users\xjtu\AppData\Roaming\Typora\typora-user-images\image-20231206111423048.png)

下载以后会得到一个安装包，然后进行解压

在解压得到的目录进行复制

```sh
sudo cp cudnn-*-archive/include/cudnn*.h /usr/local/cuda/include 
sudo cp cudnn-*-archive/lib/libcudnn* /usr/local/cuda/lib64 
sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*
```

# 4.验证安装

```python
import torch
print(torch.__version__)
print(torch.cuda.is_available())
```

如果返回True，说明安装成功

<div align="center">
  
# VTAM
Visuo-Tactile Assistive Manipulation

<img width="1110" height="479" alt="image" src="https://github.com/user-attachments/assets/c212ab2d-570e-411c-bf9f-7420078956de" />

Andnet DeBoer
</div>

build docker
```
docker build -t stretch-ai-gpu .
```
run docker
```
docker run --gpus all -it --network=host stretch-ai-gpu
```
```
docker run --gpus all -it --network=host -v ~/stretch-ai-workdir:/workspace stretch-ai-gpu:ready /bin/bash
```

```
mkdocs gh-deploy
```
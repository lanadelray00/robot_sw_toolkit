import os
import zipfile
import random
import shutil
import yaml


# 설정
zip_path = "project-2-at-2025-05-04-09-59-7cd435bc.zip" # name of the zip file
output_dir = "yolo_data" # name of the folder
train_ratio = 0.8
class_names = ['I_P']  # <-- 반드시 라벨링한 클래스명과 동일하게

# 1. 압축 해제
with zipfile.ZipFile(zip_path, 'r') as zip_ref:
    zip_ref.extractall(output_dir)

# 2. 이미지와 라벨 경로 설정
image_dir = os.path.join(output_dir, "images")
label_dir = os.path.join(output_dir, "labels")

# 3. 파일 리스트 가져오기 (.jpg 기준)
image_files = [f for f in os.listdir(image_dir) if f.endswith(".jpg")]
random.shuffle(image_files)

train_count = int(len(image_files) * train_ratio)
train_images = image_files[:train_count]
val_images = image_files[train_count:]

# 4. 폴더 구조 생성
for subset in ['train', 'val']:
    os.makedirs(os.path.join(output_dir, "images", subset), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "labels", subset), exist_ok=True)

# 5. 파일 이동
for img in train_images:
    shutil.move(os.path.join(image_dir, img), os.path.join(output_dir, "images/train", img))
    label_name = os.path.splitext(img)[0] + ".txt"
    shutil.move(os.path.join(label_dir, label_name), os.path.join(output_dir, "labels/train", label_name))

for img in val_images:
    shutil.move(os.path.join(image_dir, img), os.path.join(output_dir, "images/val", img))
    label_name = os.path.splitext(img)[0] + ".txt"
    shutil.move(os.path.join(label_dir, label_name), os.path.join(output_dir, "labels/val", label_name))

# 6. data.yaml 생성
yaml_dict = {
    'train': './images/train',
    'val': './images/val',
    'nc': len(class_names),
    'names': class_names
}

with open(os.path.join(output_dir, "data.yaml"), 'w') as f:
    yaml.dump(yaml_dict, f)

print(f"[✅ 완료] {output_dir}/ 에 학습 준비 완료되었습니다.")

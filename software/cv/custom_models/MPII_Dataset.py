import pandas as pd
import pickle
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils
import os
import cv2


class MPII_Dataset(Dataset):
    def __init__(self, annotations_file, img_dir, transform=None, label_transform=None):
        with open(annotations_file, "rb") as f:
            self.img_annoations = pickle.load(f)
        
        self.img_dir = img_dir
        self.transform = transform
        self.label_transform = label_transform

    def __len__(self):
        return len(self.img_annoations)

    def __getitem__(self, idx):
        img_path = os.path.join(self.img_dir, self.img_annoations[idx][0])
        image = cv2.COLOR_BGR2RGB(cv2.imread(img_path))
        label = self.img_annoations[idx][1]
        if self.transform:
            image = self.transform(image)
        if self.label_transform:
            label = self.label_transform(label)
        return image, label
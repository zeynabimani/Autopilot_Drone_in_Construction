# Copyright Niantic 2019. Patent Pending. All rights reserved.
#
# This software is licensed under the terms of the Monodepth2 licence
# which allows for non-commercial use only, the full terms of which are made
# available in the LICENSE file.

from __future__ import absolute_import, division, print_function

import numpy as np
import PIL.Image as pil
import matplotlib as mpl
import matplotlib.cm as cm
import base64
import io
import copy
from io import BytesIO

import torch
from torchvision import transforms, datasets

import networks
from layers import disp_to_depth
from utils import download_model_if_doesnt_exist

import random
import string

def encode_data(imageString):
    decoded_data = base64.b64decode(imageString)
    buf = io.BytesIO(decoded_data)
    img = pil.open(buf).convert('RGB')
    return img


def test_simple(input_image_string, torch, encoder, depth_decoder, device, feed_width, feed_height):
    """Function to predict for a single image or folder of images
    """
    input_image = encode_data(input_image_string)

    # PREDICTING ON EACH IMAGE IN TURN
    with torch.no_grad():
        # Load image and preprocess
        original_width, original_height = input_image.size
        input_image = input_image.resize((feed_width, feed_height), pil.LANCZOS)
        input_image = transforms.ToTensor()(input_image).unsqueeze(0)

        # PREDICTION
        input_image = input_image.to(device)
        features = encoder(input_image)
        outputs = depth_decoder(features)

        disp = outputs[("disp", 0)]
        disp_resized = torch.nn.functional.interpolate(
            disp, (original_height, original_width), mode="bilinear", align_corners=False)

        # Saving colormapped depth image
        disp_resized_np = disp_resized.squeeze().cpu().numpy()
        disp_resized_np2 = copy.deepcopy(disp_resized_np)

        y = int(disp_resized_np.shape[0] / 2)
        x = int(disp_resized_np.shape[1] / 2)
        w = 75

        center_area = disp_resized_np[y-w:y+w,x-w:x+w]
        low_values_flags = center_area < 0.38  # Where values are low
        center_area[low_values_flags] = 0  # All low values set to 0

        percentage = np.count_nonzero(center_area) / (center_area.shape[0] * center_area.shape[1])
        print(percentage)

        THRESHOLD = 0.25
        if percentage > THRESHOLD:
            vmax = np.percentile(disp_resized_np2, 95)
            normalizer = mpl.colors.Normalize(vmin=disp_resized_np2.min(), vmax=vmax)
            mapper = cm.ScalarMappable(norm=normalizer, cmap='magma')
            colormapped_im = (mapper.to_rgba(disp_resized_np2)[:, :, :3] * 255).astype(np.uint8)
            im = pil.fromarray(colormapped_im)
            # print("BASE64: " + str(len(base64.b64encode(im.tobytes()))))
            buffered = BytesIO()
            im.save(buffered, format="JPEG")
            img_str = base64.b64encode(buffered.getvalue())
            print(len(img_str))
            return True, img_str #there is near obstacle
        else:
            return False, None #the path is free


def get_random_string(length):
    letters = string.ascii_lowercase
    result_str = ''.join(random.choice(letters) for i in range(length))
    return result_str
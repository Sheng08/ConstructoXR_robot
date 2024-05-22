from PIL import Image

def get_image_size(image_path):
    with Image.open(image_path) as img:
        return img.size  # 返回格式 (width, height)

image_path = 'output.pgm'
size = get_image_size(image_path)
print("Width:", size[0], "Height:", size[1])

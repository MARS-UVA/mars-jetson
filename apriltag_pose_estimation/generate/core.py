from collections.abc import Sequence
from importlib.resources import files
from pathlib import Path

try:
    from fpdf import FPDF
    from fpdf.enums import Align
    from PIL import Image
except ImportError as e:
    e.add_note('to use PDF generation features, install this package with the "generate" extra')
    raise

from .. import tag_images


MM_OVER_PIXELS = 127 / 480


def get_tag_image(tag_id: int, tag_size: float) -> Image.Image:
    """
    Gets an appropriately-sized image of the AprilTag with the given ID.

    AprilTags will be taken from the tagStandard41h12 family.

    The actual size of the image will be about twice the given size to ensure the image will be high-quality when
    scaled to the given size.

    :param tag_id: The ID of the AprilTag to use.
    :param tag_size: The distance between the tag's corner points in millimeters. For the family being used, the corner
                     points are the corners of the white interior square.
    :return: A PIL Image of the tag at a size easily sufficient to render the tag at high quality at the provided size.
    """
    tag_size_pixels = round(tag_size / MM_OVER_PIXELS) * 2
    with Image.open(str(files(tag_images).joinpath(f'{tag_id}.png'))) as img:
        # noinspection PyUnresolvedReferences
        return img.resize(size=(tag_size_pixels, tag_size_pixels), resample=Image.BOX)


def generate_apriltag_pdf(tag_ids: Sequence[int], tag_size: float, outfile: Path = Path('tags.pdf')) -> None:
    """
    Generates a PDF file with the given AprilTags.

    The AprilTag family being used is the tagStandard41h12 family.

    :param tag_ids: A sequence of AprilTag IDs. Currently only a limited number of IDs are supported.
    :param tag_size: The distance between the tag's corner points in millimeters. For the family being used, the corner
                     points are the corners of the white interior square.
    :param outfile: The path to which the resulting PDF will be written.
    """
    tag_image_size = tag_size * 9 / 5

    pdf = FPDF(orientation='portrait', unit='mm', format='letter')
    pdf.set_font(family='Helvetica', size=32)

    center_x = pdf.w / 2
    center_y = pdf.h / 2
    image_center_x = (pdf.w - tag_image_size) / 2
    image_center_y = (pdf.h - tag_image_size) / 2
    marker_distance_from_x_edge = image_center_x - 5
    marker_distance_from_y_edge = image_center_y - 5
    for tag_id, tag_img in ((tag_id, get_tag_image(tag_id, tag_image_size)) for tag_id in tag_ids):
        pdf.add_page(same=True)
        # Draw the tag
        pdf.image(tag_img, x=image_center_x, y=image_center_y, w=tag_image_size, h=tag_image_size,
                  keep_aspect_ratio=True)
        # Draw the text label
        pdf.set_y(pdf.h - marker_distance_from_y_edge + 15)
        pdf.cell(w=tag_image_size, text=f'ID: {tag_id} - Size: {tag_size} mm', align=Align.C, center=True)
        # Draw alignment markers
        pdf.line(marker_distance_from_x_edge - 10, center_y, marker_distance_from_x_edge, center_y)
        pdf.line(pdf.w - (marker_distance_from_x_edge - 10), center_y, pdf.w - marker_distance_from_x_edge, center_y)
        pdf.line(center_x, marker_distance_from_y_edge - 10, center_x, marker_distance_from_y_edge)
        pdf.line(center_x, pdf.h - (marker_distance_from_y_edge - 10), center_x, pdf.h - marker_distance_from_y_edge)

    pdf.set_title('Field AprilTags')
    pdf.set_creator('apriltag_pose_estimation.generate')
    pdf.output(str(outfile))

import os
from collections.abc import Sequence
from pathlib import Path
from typing import Literal

from ..core.bindings import AprilTagFamilyId

try:
    from fpdf import FPDF
    from fpdf.enums import Align
    from PIL import Image
except ImportError as e:
    e.add_note('to use PDF generation features, install this package with the "generate" extra')
    raise

from ..apriltag.render.image import AprilTagImageGenerator

MM_OVER_PIXELS = 127 / 480


def generate_apriltag_pdf(tag_ids: Sequence[int], tag_size: float,
                          tag_family: AprilTagFamilyId = 'tagStandard41h12',
                          page_format: Literal['a3', 'a4', 'a5', 'letter', 'legal'] | tuple[int | float, int | float] = 'letter',
                          outfile: Path = Path('tags.pdf'),
                          search_paths: Sequence[str | os.PathLike] = (
                                  Path(__file__).parent / 'lib',
                                  Path(__file__).parent / 'lib64'
                          )) -> None:
    """
    Generates a PDF file with the given AprilTags.

    The AprilTag family being used is the tagStandard41h12 family.

    :param tag_ids: A sequence of AprilTag IDs. Currently only a limited number of IDs are supported.
    :param tag_size: The distance between the tag's corner points in millimeters. For the family being used, the corner
                     points are the corners of the white interior square.
    :param outfile: The path to which the resulting PDF will be written.
    """
    tag_image_size = tag_size * 9 / 5

    pdf = FPDF(orientation='portrait', unit='mm', format=page_format)
    pdf.set_font(family='Helvetica', size=32)
    pdf.set_auto_page_break(False)

    center_x = pdf.w / 2
    center_y = pdf.h / 2
    image_center_x = (pdf.w - tag_image_size) / 2
    image_center_y = (pdf.h - tag_image_size) / 2
    marker_distance_from_x_edge = image_center_x - 5
    marker_distance_from_y_edge = image_center_y - 5
    if marker_distance_from_x_edge - 5 < 6 or marker_distance_from_y_edge - 5 < 6:
        raise ValueError('Tag is too big to fit all graphics')
    marker_distance_from_x_edge_closer = max(6, marker_distance_from_x_edge - 10)
    marker_distance_from_y_edge_closer = max(6, marker_distance_from_y_edge - 10)

    apriltag_image_generator = AprilTagImageGenerator(family=tag_family, search_paths=search_paths)

    for tag_id, tag_img in ((tag_id,
                             Image.fromarray(apriltag_image_generator.generate_image(tag_id, 2 * round(tag_image_size / MM_OVER_PIXELS))))
                            for tag_id in tag_ids):
        pdf.add_page(same=True)
        # Draw the tag
        pdf.image(tag_img, x=image_center_x, y=image_center_y, w=tag_image_size, h=tag_image_size,
                  keep_aspect_ratio=True)
        # Draw the text label
        pdf.set_y(pdf.h - marker_distance_from_y_edge + 15)
        pdf.cell(w=tag_image_size, text=f'ID: {tag_id} - Size: {tag_size} mm', align=Align.C, center=True)
        # Draw alignment markers
        pdf.line(marker_distance_from_x_edge_closer, center_y, marker_distance_from_x_edge, center_y)
        pdf.line(pdf.w - marker_distance_from_x_edge_closer, center_y, pdf.w - marker_distance_from_x_edge, center_y)
        pdf.line(center_x, marker_distance_from_y_edge_closer, center_x, marker_distance_from_y_edge)
        pdf.line(center_x, pdf.h - marker_distance_from_y_edge_closer, center_x, pdf.h - marker_distance_from_y_edge)

    pdf.set_title('Field AprilTags')
    pdf.set_creator('apriltag_pose_estimation.generate')
    pdf.output(str(outfile))

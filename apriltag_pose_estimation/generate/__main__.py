import argparse
from pathlib import Path

from .core import generate_apriltag_pdf


def process_desired_tags(desired_tags_string: str) -> list[int]:
    """
    Processes the string the user inputs for the tags they wish to include in the PDF.

    The string is either a single ID, a range of IDs, or a comma-separated list of IDs and/or ID ranges. ID ranges
    are indicated with a hyphen (``-``), and the start and end IDs are included in the range. Tag IDs may be repeated
    and will be returned in the order they were specified.
    """
    tag_ids: list[int] = []
    for id_or_range in (s.rstrip() for s in desired_tags_string.split(',')):
        if '-' in id_or_range:
            limits = id_or_range.split('-')
            if len(limits) != 2:
                raise ValueError(f'invalid tag id range: {id_or_range}')
            try:
                tag_ids.extend(range(int(limits[0]), int(limits[1]) + 1))
            except ValueError as e:
                raise ValueError(f'invalid tag id range: {id_or_range}') from e
        else:
            try:
                tag_ids.append(int(id_or_range))
            except ValueError as e:
                raise ValueError(f'invalid tag id: {id_or_range}') from e
    return tag_ids


def main() -> None:
    parser = argparse.ArgumentParser(prog='generate',
                                     description='Generate printable PDFs for AprilTags',
                                     epilog='The AprilTag family being used is the tagStandard41h12 family.')
    parser.add_argument('--ids', dest='tag_ids', required=True, type=process_desired_tags,
                        help='IDs of the tags which will be included in the PDF. Can be specified as a single ID '
                             '(e.g., "4"), a range of IDs (e.g., "0-7"), or a comma-separated list of IDs and/or ID '
                             'ranges (e.g., "0,3-5,2").')
    parser.add_argument('--size', dest='tag_size', required=True, type=float,
                        help='Distance between the tags\' corner points in millimeters. For the family being used, the '
                             'corner points are the corners of the white interior square.')
    parser.add_argument('--page-format', dest='page_format', type=str, default='letter',
                        choices=['a3', 'a4', 'a5', 'letter', 'legal', 'ledger'],
                        help='Distance between the tags\' corner points in millimeters. For the family being used, the '
                             'corner points are the corners of the white interior square.')
    parser.add_argument('-o', '--outfile', dest='outfile', required=False, type=Path, default='tags.pdf',
                        help='Path to the output file. Should have extension .pdf (default: tags.pdf).')

    args = parser.parse_args()

    generate_apriltag_pdf(tag_ids=args.tag_ids,
                          tag_size=args.tag_size,
                          page_format=(279.4, 431.8) if args.page_format == 'ledger' else args.page_format,
                          outfile=args.outfile,
                          search_paths='.')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import os
import subprocess
from collections.abc import Generator
from contextlib import contextmanager
from pathlib import Path
from tempfile import TemporaryDirectory


@contextmanager
def change_directory(path: Path) -> Generator[None, None, None]:
    old_path = Path.cwd()
    try:
        os.chdir(path)
        yield
    finally:
        os.chdir(old_path)


def install_dependency(install_dir: Path,
                       name: str,
                       tag: str,
                       remote_url: str) -> None:
    with TemporaryDirectory() as tmpdir:
        temp_path = Path(tmpdir).absolute()
        project_dir = temp_path / name
        subprocess.run(['git', 'clone',
                        '--recurse-submodules',
                        '--depth', '1',
                        '--branch', tag,
                        '--single-branch',
                        remote_url,
                        project_dir])
        build_dir = project_dir / 'build'
        build_dir.mkdir()
        with change_directory(build_dir):
            subprocess.run(['cmake',
                            f'-DCMAKE_INSTALL_PREFIX:PATH={install_dir}',
                            project_dir])
            subprocess.run(['make', 'install'])


def main() -> None:
    dependencies: list[tuple[str, str, str]] = [
        ('apriltag', 'v3.4.5', 'https://github.com/AprilRobotics/apriltag.git'),
    ]
    project_dir = Path(__file__).parent.absolute()
    for name, tag, remote_url in dependencies:
        install_dependency(install_dir=project_dir, name=name, tag=tag, remote_url=remote_url)


if __name__ == '__main__':
    main()

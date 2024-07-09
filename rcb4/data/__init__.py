from collections import namedtuple
import os
import os.path as osp

from colorama import Fore
from colorama import Style
import gdown
import subprocess


data_dir = osp.abspath(osp.dirname(__file__))
_default_cache_dir = osp.expanduser('~/.rcb4')


ELFINFO = namedtuple('ELFINFO', ['url', 'md5sum'])

elf_infos = {
    'v0.6.2': ELFINFO(
        'https://github.com/iory/rcb4/releases/download/v0.6.2.elf/v0.6.2.elf',  # NOQA
        '9205b34ce4d81b87c4f11e6139f96b17'),
    'v0.6.2.pump': ELFINFO(
        'https://github.com/nakane11/rcb4/releases/download/v0.6.2.pump/v0.6.2.pump.elf',  # NOQA
        '0b5ee7f7300c622d76bba0b572415b2d'),
    'v0.6.3': ELFINFO(
        'https://github.com/nakane11/rcb4/releases/download/v0.6.3/v0.6.3.elf',
        'bdf7777c8cd3cb19398d8968b8714896'),
}


def get_cache_dir():
    return os.environ.get('RCB4_CACHE_DIR', _default_cache_dir)


def kondoh7_elf(version='v0.6.3'):
    if version not in elf_infos:
        raise RuntimeError(
            'Invalid armh7 version. Valid versions are {}'
            .format(list(elf_infos.keys())))
    elf_info = elf_infos[version]
    target_path = osp.join(get_cache_dir(), 'elf', version + '.elf')
    try:
        gdown.cached_download(
            url=elf_info.url,
            path=target_path,
            md5=elf_info.md5sum,
            quiet=True)
    except Exception as e:
        print(Fore.RED + str(e))
        print('Download elf file failed. \nPlease download '
              + Fore.BLUE + elf_info.url + Fore.RED
              + ' and put it to ' + target_path)
        print(Style.RESET_ALL)
    return target_path


def stlink():
    st_flash_path = osp.join(get_cache_dir(), 'stlink', 'stlink-1.7.0',
                             'build', 'Release', 'bin', 'st-flash')
    if osp.exists(st_flash_path):
        return st_flash_path
    url = 'https://github.com/stlink-org/stlink/archive/refs/tags/v1.7.0.tar.gz'  # NOQA
    md5sum = '583a506c8e5e65577d623b5ace992fe5'
    target_path = osp.join(get_cache_dir(), 'stlink', 'v1.7.0.tar.gz')
    target_dir = osp.join(get_cache_dir(), 'stlink', 'stlink-1.7.0')
    gdown.cached_download(
        url=url,
        path=target_path,
        md5=md5sum,
        quiet=True,
        postprocess=gdown.extractall)
    ret = subprocess.run(f'cd {target_dir} && make',
                         shell=True,
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    if ret.returncode == 0:
        return st_flash_path
    else:
        raise RuntimeError('Building stlink failed. Notify to maintainer.')

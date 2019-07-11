from os.path import dirname, basename, isfile
import glob

ext = '.py'
ext_length = len(ext)

dir_path = dirname(__file__) + '/*' + ext

modules = glob.glob(dir_path)

__all__ = [ basename(f)[:-ext_length] for f in modules if isfile(f) and not f.endswith('__init__.py') ]
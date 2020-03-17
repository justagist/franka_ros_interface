#! /bin/bash

# NOTE: if googleanalytics fails with this error, (exception: cannot import name ExtensionError),
# run the last 6 lines of this file (from the root of this file)

sudo pip install sphinx

## ----- install sphinx-contrib googleanalytics
hg clone https://bitbucket.org/birkenfeld/sphinx-contrib/ ../docs_build/sphinx-contrib
cd ../docs_build/sphinx-contrib/googleanalytics/
python setup.py build
sudo python setup.py install


cd ../../../docs_src/
sudo pip install sphinx_rtd_theme


## ----- for latexpdf
sudo apt install latexmk texlive-xetex






## --------------------------------------------------------------------
## To fix (exception: cannot import name ExtensionError) error, 
## uncomment and run the following

# cd ../docs_build/sphinx-contrib/googleanalytics/
# original="from sphinx.application import ExtensionError"
# new="from sphinx.errors import ExtensionError"
# sed -i -e "s/$original/$new/g" sphinxcontrib/googleanalytics.py
# python setup.py build
# sudo python setup.py install
# cd ../../../docs_src/

## ----------------------------------------------------------------------
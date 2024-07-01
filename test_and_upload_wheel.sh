!/bin/bash
set -e -u -x

# Install packages and test
for PYBIN in /opt/python/*/bin; do
  if [[ "${PYBIN}" != *cp311* ]] && [[ "${PYBIN}" != *cp312* ]] && [[ "${PYBIN}" != *cp313* ]] && [[ "${PYBIN}" != *cp36* ]] && [[ "${PYBIN}" != *cp37* ]] && [[ "${PYBIN}" != *py37* ]] && [[ "${PYBIN}" != *cp38* ]] && [[ "${PYBIN}" != *py38* ]] && [[ "${PYBIN}" != *py39* ]] && [[ "${PYBIN}" != *cp310* ]] && [[ "${PYBIN}" != *py310* ]]; then
    "${PYBIN}/pip" install opensfm --no-index -f $WHEEL_DIR
    cd / && "${PYBIN}/python" -c "import opensfm"
  fi
done


for WHL in $WHEEL_DIR/*.whl; do
    /opt/python/cp39-cp39/bin/python -m twine upload --repository-url "http://pypi.artichoke-labs.ai" $WHL
done

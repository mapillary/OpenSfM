!/bin/bash
set -e -u -x

function repair_wheel {
    wheel="$1"
    if ! auditwheel show "$wheel"; then
        echo "Skipping non-platform wheel $wheel"
    else
        auditwheel repair "$wheel" --plat "$PLAT" -w $WHEEL_DIR
    fi
}

# Compile wheels
for PYBIN in /opt/python/*/bin; do
  if [[ "${PYBIN}" != *cp311* ]] && [[ "${PYBIN}" != *cp312* ]] && [[ "${PYBIN}" != *cp313* ]] && [[ "${PYBIN}" != *cp36* ]] && [[ "${PYBIN}" != *cp37* ]] && [[ "${PYBIN}" != *py37* ]] && [[ "${PYBIN}" != *cp38* ]] && [[ "${PYBIN}" != *py38* ]] && [[ "${PYBIN}" != *py39* ]] && [[ "${PYBIN}" != *py310* ]]; then
    "${PYBIN}/pip" install --upgrade pip
    "${PYBIN}/pip" install -r ${SFM_DIR}/requirements.txt
    "${PYBIN}/pip" wheel $SFM_DIR --no-deps -w $WHEEL_DIR
  fi
done

# Bundle external shared libraries into the wheels
for whl in $WHEEL_DIR/*.whl; do
    repair_wheel "$whl"
done

# Install packages and test
for PYBIN in /opt/python/*/bin; do
  if [[ "${PYBIN}" != *cp311* ]] && [[ "${PYBIN}" != *cp312* ]] && [[ "${PYBIN}" != *cp313* ]] && [[ "${PYBIN}" != *cp36* ]] && [[ "${PYBIN}" != *cp37* ]] && [[ "${PYBIN}" != *py37* ]] && [[ "${PYBIN}" != *cp38* ]] && [[ "${PYBIN}" != *py38* ]] && [[ "${PYBIN}" != *py39* ]] && [[ "${PYBIN}" != *py310* ]]; then
    "${PYBIN}/pip" install opensfm --no-index -f $WHEEL_DIR
#    opensfm --help
  fi
done


# Need to update this code
/opt/python/cp39-cp39/bin/python -m twine upload --repository-url "http://pypi.artichoke-labs.ai" $WHEEL_DIR/opensfm-0.5.2.post6-cp39-cp39-linux_x86_64.whl
/opt/python/cp310-cp310/bin/python -m twine upload --repository-url "http://pypi.artichoke-labs.ai" $WHEEL_DIR/opensfm-0.5.2.post6-cp310-cp310-linux_x86_64.whl
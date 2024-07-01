!/bin/bash
set -e -u -x

function repair_wheel {
    wheel="$1"
    if ! /opt/_internal/cpython-3.9.19/bin/auditwheel show "$wheel"; then
        echo "Skipping non-platform wheel $wheel"
    else
        /opt/_internal/cpython-3.9.19/bin/auditwheel repair "$wheel" --plat "$PLAT" -w $WHEEL_DIR
    fi
}

# Compile wheels
for PYBIN in /opt/python/*/bin; do
  if [[ "${PYBIN}" != *cp311* ]] && [[ "${PYBIN}" != *cp312* ]] && [[ "${PYBIN}" != *cp313* ]] && [[ "${PYBIN}" != *cp36* ]] && [[ "${PYBIN}" != *cp37* ]] && [[ "${PYBIN}" != *py37* ]] && [[ "${PYBIN}" != *cp38* ]] && [[ "${PYBIN}" != *py38* ]] && [[ "${PYBIN}" != *py39* ]] && [[ "${PYBIN}" != *cp310* ]] && [[ "${PYBIN}" != *py310* ]]; then
    "${PYBIN}/pip" install --upgrade pip
    "${PYBIN}/pip" install -r ${SFM_DIR}/requirements.txt
    "${PYBIN}/pip" wheel $SFM_DIR --no-deps -w $WHEEL_DIR
  fi
done

# Bundle external shared libraries into the wheels
for whl in $WHEEL_DIR/*.whl; do
    repair_wheel "$whl"
done

cd ${WHEEL_DIR} && rm -rf *-linux*whl

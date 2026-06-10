# pyre-strict
"""Regression tests for opensfm.feature_loading.FeatureLoader."""

import inspect

from opensfm import feature_loading


def test_clear_cache_clears_all_lru_cache_methods() -> None:
    """clear_cache() must invoke cache_clear() on every @lru_cache method.

    Without this, callers that switch to a new dataset and rely on
    clear_cache() to release the previous dataset's cached bearings end up
    with the previous DataSet pinned via the lru_cache's strong reference
    on the data argument.
    """
    loader = feature_loading.FeatureLoader()

    cached_method_names = sorted(
        name
        for name in dir(loader)
        if not name.startswith("__")
        and callable(getattr(loader, name, None))
        and hasattr(getattr(loader, name), "cache_clear")
        and hasattr(getattr(loader, name), "cache_info")
    )
    assert cached_method_names, (
        "expected FeatureLoader to expose @lru_cache-decorated methods"
    )

    src = inspect.getsource(loader.clear_cache)
    missing = [
        name for name in cached_method_names if f"self.{name}.cache_clear()" not in src
    ]
    assert not missing, (
        f"FeatureLoader.clear_cache() forgets to clear these @lru_cache methods: "
        f"{missing}. Add `self.<method>.cache_clear()` for each."
    )

from opensfm import csfm


def test_unicode_strings_in_bundle():
    """Test that byte and unicode strings can be used as camera ids."""
    ba = csfm.BundleAdjuster()

    unicode_id = u"A\xb2"
    byte_id = b"A_2"

    ba.add_equirectangular_camera(unicode_id)
    ba.add_equirectangular_camera(byte_id)

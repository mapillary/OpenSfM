import yaml
default_config_yaml = '''
# Metadata
extract_features: True          # False = load from disk

refine_with_local_map: True
tracker_lk: False
match_symm: True

run_local_ba_every_nth: 1

# Feature parameters
feat_cell_size: 64
feat_cell_overlap: 6
feat_pyr_levels: 8
feat_scale: 1.2
feat_max_number: 4000

# FAST
feat_fast_ini_th: 20
feat_fast_min_th: 7

# grid parameters (from OpenVSlam)
grid_n_cols: 64
grid_n_rows: 48


'''

def default_config():
    """Return default configuration"""
    return yaml.safe_load(default_config_yaml)

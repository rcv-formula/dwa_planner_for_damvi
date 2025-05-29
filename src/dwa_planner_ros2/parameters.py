import yaml

def declare_parameters(node):
    config_file = node.declare_parameter('config_file', 'config/params.yaml').value
    with open(config_file, 'r') as f:
        params = yaml.safe_load(f)
    for name, value in params.items():
        node.declare_parameter(name, value)

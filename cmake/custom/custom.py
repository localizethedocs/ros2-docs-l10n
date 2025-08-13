import json
import os

# Default configuration values
DEFAULT_CONFIG_VALUES = {
    "html_baseurl"        : "",
    "latest_version"      : "",
    "current_version"     : "",
    "current_language"    : "",
    "versions_json_path"  : "versions.json",
}

def add_default_config_values(app):
    """
    Add default configuration values to the Sphinx app if not already defined.
    """
    for key, default in DEFAULT_CONFIG_VALUES.items():
        if key not in app.config.values:
            app.add_config_value(key, default, "env")

def load_versions(app, filepath):
    """
    Load the versions.json and generate html_context variables.
    """
    if filepath and os.path.isfile(filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

            latest_version_name = app.config.latest_version

            current_version_name = app.config.current_version

            versions_data = {
                "releases": data.get("releases", []),
                "in_development": data.get("in_development", [])
            }

            eol_versions = [
                version["name"]
                for version in data.get("releases", [])
                if version.get("eol", False)
            ]

            latest_version_obj = next(
                (v for v in data.get("releases", []) + data.get("in_development", []) if v["name"] == latest_version_name),
                None
            )

            current_version_obj = next(
                (v for v in data.get("releases", []) + data.get("in_development", []) if v["name"] == current_version_name),
                None
            )

            app.config.html_context["versions"] = versions_data
            app.config.html_context["eol_versions"] = eol_versions
            app.config.html_context["latest_version"] = latest_version_obj
            app.config.html_context["current_version"] = current_version_obj

            print("[DEBUG] html_context['versions']:", json.dumps(versions_data, indent=2, ensure_ascii=False))
            print("[DEBUG] html_context['eol_versions']:", eol_versions)
            print("[DEBUG] html_context['latest_version']:", json.dumps(latest_version_obj, indent=2, ensure_ascii=False))
            print("[DEBUG] html_context['current_version']:", json.dumps(current_version_obj, indent=2, ensure_ascii=False))

def setup(app):
    """
    Sphinx extension entry point.
    """
    add_default_config_values(app)

    def on_config_inited(app, config):
        app.config.html_context["html_baseurl"] = config.html_baseurl
        app.config.html_context["current_language"] = app.config.current_language
        load_versions(app, config.versions_json_path)

    app.connect("config-inited", on_config_inited)

    return {
        "parallel_read_safe": True,
        "parallel_write_safe": True,
    }

import os
import sys
import argparse
import cip
import yaml
from collections import defaultdict

__script_path = os.path.abspath(os.path.dirname(__file__))
def get_script_path():
    return __script_path

def get_repo_root_path():
    return os.path.abspath(os.path.join(get_script_path(), "../"))

def stage_sync():
    cip.main(["--debug", "sync"])

def stage_config(buildYmlConfFilePath):
    with open(buildYmlConfFilePath, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            sys.exit(exc)

def stage_fetch(buildYmlConf, fetch_projects, destination):
    fetched_cache = defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))
    for mf_sil_variant in buildYmlConf["variants"]:
        for fetch_project in buildYmlConf["variants"][mf_sil_variant]["uses"]:
            fetch_project_platforms = buildYmlConf["variants"][mf_sil_variant]["build_platforms"]
            if fetch_project in fetch_projects:
                for fetch_project_platform in fetch_project_platforms:
                    fetch_project_conf = buildYmlConf["variants"][mf_sil_variant]["uses"][fetch_project]
                    if fetched_cache.get(fetch_project,{}).get(fetch_project_platform,{}).get(fetch_project_conf["variant"], {}).get(fetch_project_conf["release"]):
                        print("Already fetched {}".format("".join([fetch_project_platform, "/", fetch_project, ".", fetch_project_conf["variant"], "-", fetch_project_conf["release"]])))
                    else:
                        cip.main(["download",
                                "--package", fetch_project,
                                "--variant", fetch_project_conf["variant"],
                                "--platform", fetch_project_platform,
                                "--version", fetch_project_conf["release"],
                                "--destination", os.path.join(destination, fetch_project_platform, fetch_project + "." + fetch_project_conf["variant"])])
                        fetched_cache[fetch_project][fetch_project_platform][fetch_project_conf["variant"]][fetch_project_conf["release"]] = True

def main(argv):
    parser = argparse.ArgumentParser(
        description="""Script to fetch packages from Bricks.""")
    parser.add_argument("-s", "--sync", help="Run cip sync first", action='store_true')
    parser.add_argument('-p', '--projects', nargs='+', default=["us_processing"])
    parser.add_argument("-d", "--destination", help="Output destination for fetched packages. (default: %(default)s)", default=os.path.join(get_repo_root_path(),"contrib", "cip"))
    parser.add_argument("-c", "--config", help="Location of build.yml (default: %(default)s)", default=os.path.join(get_repo_root_path(),"conf", "build.yml"))
    args = parser.parse_args()

    if args.sync:
        stage_sync()

    buildYmlConf = stage_config(args.config)

    stage_fetch(buildYmlConf, args.projects, args.destination)

if __name__ == "__main__":
    main(sys.argv[:])

import yaml

gitHubDefaultPath = "http://github-am.geo.conti.de/ADAS/"

# read yaml file
with open("conf/build.yml", 'r') as stream:
	data_loaded = yaml.safe_load(stream)
	
#print(data_loaded)

dependencies = data_loaded.get('variants').get('generic').get('uses')
print(dependencies)

for key in dependencies:
	repo = key
	release = dependencies[key].get('release')
	row = [repo, gitHubDefaultPath+repo, release]
	print row

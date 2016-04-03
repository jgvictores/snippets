# Manually install fixmystreet on Ubuntu
## Install dependencies from Ubuntu official repositories
```
setxkbmap es,es  # Change keyboard to layout if required, e.g. spanish.
sudo apt-get install tasksel  # Required for next step; you may be asked your sudo password.
sudo tasksel install lamp-server  # You may be asked to set mysql root pass: mysqlPassword
sudo apt-get install postgresql postgresql-contrib # Required postgres database server.
sudo apt-get install postgresql-client-common postgresql-client  # Required postgres database client.
sudo apt-get install libapache2-mod-auth-pgsql  # Connect apache and postgresql (perhaps not required).
sudo apt-get install ruby  # Required for gem.
sudo apt-get install git  # Be able to download extra repositories.
```

## Download fixmystreet and extra dependencies
```
mkdir FixMyStreet  # Make a directory for fixmystreet and extra dependencies.
cd FixMyStreet  # Enter this directory.
git clone --recursive https://github.com/mysociety/fixmystreet.git  # Download fixmystreet with "recursive" flag to get embedded repositories too.
cd fixmystreet  # Enter its directory.
sudo ./bin/install_perl_modules  # Launch this; it takes a while.
sudo gem install --user-install --no-ri --no-rdoc bundler  # Launch this.
$(ruby -rubygems -e 'puts Gem.user_dir')/bin/bundle install --deployment --path ../gems --binstubs ../gem-bin  # Launch this.
./bin/make_css  # Launch this.
```

## Create postgres database user
```
sudo -u postgres psql  # Enter the database.
```
Within the database, enter the following commands. You may change *postgresPassword* for that of your choice.
```
CREATE USER fms WITH PASSWORD 'postgresPassword';
CREATE DATABASE fms WITH OWNER fms;
\c fms
CREATE LANGUAGE plpgsql;
\q
```
The final comand exits the database.

## Modify postgres database permissions
```
sudo updatedb  # This updates the indexed search used by "locate".
sudo gedit `locate main/pg_hba.conf`  # This opens a text editor.
```
In the open text editor, add the following line as the first line of the file.
```
local   fms     fms     md5
```
Save and close the text editor. Now restart the postgres database server.
```
sudo service postgresql restart
```
## Create the postgres database schema
You will be asked for the postgres database user password for each of the 3 commands. Insert the *postgresPassword* you fixed previously.
```
psql -U fms fms < db/schema.sql
psql -U fms fms < db/generate_secret.sql
psql -U fms fms < db/alert_types.sql
```

## Configure fixmystreet parameters
```
cp conf/general.yml-example conf/general.yml  # Copy the parameter file template to the one that is actually parsed.
gedit conf/general.yml  # This opens a text editor.
```
Within the text editor, modify the following lines (insert the *postgresPassword* you fixed previously).
```
FMS_DB_PASS: 'postgresPassword'
BASE_URL: 'http://localhost:3000/'
MAPIT_URL: 'http://localhost:3000/fakemapit/'
```
Save and close the text editor.

## Set up some required data
```
./bin/update-all-reports
./commonlib/bin/gettext-makemo FixMyStreet
```

## Run
```
script/fixmystreet_app_server.pl -d --fork  # Will be available on localhost:3000
```

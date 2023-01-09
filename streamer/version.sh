while true;
do
  git describe --tags --abbrev=0 | nc -k -l -p 80 -q 1
done

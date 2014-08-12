# Converts protobuf .proto files to ros .msg files
# Author: Harry J.E Day
# Date:   9/08/14
#! 

echo $
pwd
echo "Converting proto to msg $1"

sed -r "/^package \w+; *$/d" $1 |
sed -r "s/^.*(\{|\}).*$/ /"  |
sed -r "s/double/float64/"  |
sed  -r "s/= *[0-9]*;//" | sed "s/repeated//" | sed -r "s/^$//"  > $2
    

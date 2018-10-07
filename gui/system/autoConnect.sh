i=`ping -w 1 -c 1 192.168.1.10`
while test ! $? -eq 0
do
   echo "PING"
   i=`ping -w 1 -c 1 192.168.1.10`
done

`ssh -X ros@192.168.1.10`
read line

if test $line = "Enter Password:"
then
      echo "ros\n"
else
do
   `ssh -X bluenuc@192.168.1.10`
   read  line
   if test $line = "Enter Password"
   then
      echo "ros\n"
   fi
fi

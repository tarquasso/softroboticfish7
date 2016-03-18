
#!/bin/sh

# contains(string, substring)
#
# Returns 0 if the specified string contains the specified substring,
# otherwise returns 1.
contains() {
    string="$1"
    substring="$2"
    if test "${string#*$substring}" != "$string"
    then
        return 0    # $substring is in $string
    else
        return 1    # $substring is not in $string
    fi
}

#contains "abcd" "d" && echo "abcd does not contain e"


VOLUME=165 # Default volume
if [ "$#" -eq 1 ]; then
  VOLUME=$1
  PERCENT=1
  contains "$1" "%" && VOLUME=`echo $VOLUME | cut -b 1-2` && VOLUME=$((VOLUME*255/100))
  echo "VOLUME" $VOLUME
fi

echo "Setting volume to" $VOLUME "("$((VOLUME*100/215))"%)"
amixer -c 0 set Digital $VOLUME

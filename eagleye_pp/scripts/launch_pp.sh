#!/bin/bash

EAGLEYE_PP="./devel/lib/eagleye_pp/eagleye_pp"

function usage() {
cat <<_EOT_
Usage:
  $0 <BAG_FILES> <CONFIG>

Description:
  Eagleye post process

Options:
  None

_EOT_

exit 1
}

if [ "$OPTIND" = 1 ]; then
  while getopts h OPT
  do
    case $OPT in
      h)
        usage ;;
      \?)
        echo "Undefined option $OPT"
        usage ;;
    esac
  done
else
  echo "No installed getopts-command." 1>&2
  exit 1
fi

shift $(($OPTIND - 1))

# Set ROSBAGs
ARGC=$#
ARGV=("$@")

# Total number of input ROSBAGs
N_BAG=$(($ARGC-1))

# Prepare ROSBAG names
BAG_FILES=""
for (( i=0; i<N_BAG; i++ ))
do
  BAG_FILES+="${ARGV[i]}"" "
done
BAG_FILES="$(echo -e "${BAG_FILES}" | sed -e 's/[[:space:]]*$//')"

# Parse other arguments
TMP_CONFIG=${ARGV[$(($ARGC-1))]}
CONFIG=$(cd $(dirname $TMP_CONFIG); pwd)/$(basename $TMP_CONFIG)

DATE=$(date +%Y%m%d-%H%M%S)

# Set filename
FILE_NAME="${ARGV[0]%.*}"
FILE_PATH=$(cd $(dirname ${ARGV[0]}) && pwd)/$(basename ${ARGV[0]})
FILE_PATH="${FILE_PATH%/*}/${DATE}/"

mkdir $FILE_PATH

COPIED_CONFIG=${FILE_PATH}"config.yaml"

echo "BAG_FILES: "$BAG_FILES
echo "FILE_PATH: "$FILE_PATH
echo "CONFIG: "$CONFIG 

$EAGLEYE_PP $N_BAG $BAG_FILES $FILE_PATH $CONFIG

cp $CONFIG $COPIED_CONFIG

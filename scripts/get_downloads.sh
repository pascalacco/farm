#!/bin/bash

function uncompress
{
    if [[ $1 == *.bz2 ]]; 
    then
	mkdir $2
        tar jxf $1 -C $2 --strip-components=1
    fi

    if [[ $1 == *.tgz ]]; 
    then
	mkdir $2
        tar xf $1 -C $2 --strip-components=1
    fi

    if [[ $1 == *.gz ]]; 
    then
	mkdir $2
        tar xf $1 -C $2 --strip-components=1
    fi

    if [[ $1 == *.zip ]]; 
    then
        unzip -q $1 -d $2
    fi
}

function get_it
{

    echo "_______________________"
    echo "Get   $1            "
    echo "_______________________"

    echo "check if it's already unpacked in "$4/$5" ..."
    if [ -f $4/$5 ] || [ -d $4/$5 ] ;
    then
	echo "   it's already there"
	echo "Done"
    else
	echo "   NO, check if archive file " ${DOWNLOAD_DIR}/$2 "exists..."
	if [ ! -f ${DOWNLOAD_DIR}/$2 ]
	then
	    echo "      NO, fetching from web " $3 " ..."
	    wget -O ${DOWNLOAD_DIR}/$2 $3
	    echo "      Done,"
	else
	    echo "      YES"
	    echo "   Done,"
	fi
    
	echo "   uncompressing to directory " $4 " ..."
	uncompress ${DOWNLOAD_DIR}/$2 $4
	echo "Done"
    fi
}


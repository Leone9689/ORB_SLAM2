all:
	cd Thirdparty/DBoW2/build/; make
	cd Thirdparty/g2o/build/; make
	cd build; make

clean:
	cd build; make clean
	cd Thirdparty/g2o/build/; make clean
	cd Thirdparty/DBoW2/build/; make clean

RPMBUILD = rpmbuild --define "_topdir %(pwd)/build" \
        --define "_builddir %{_topdir}" \
        --define "_rpmdir %{_topdir}" \
        --define "_srcrpmdir %{_topdir}" \
        --define "_sourcedir %(pwd)"

GIT_VERSION = $(shell git name-rev --name-only --tags --no-undefined HEAD 2>/dev/null || echo git-`git rev-parse --short HEAD`)
SERVER_VERSION=$(shell awk '/Version:/ { print $$2; }' observatory-raptor-camera-server.spec)

all:
	mkdir -p build
	cp raptor_camd raptor_camd.bak
	awk '{sub("SOFTWARE_VERSION = .*$$","SOFTWARE_VERSION = \"$(SERVER_VERSION) ($(GIT_VERSION))\""); print $0}' raptor_camd.bak > raptor_camd
	${RPMBUILD} -ba observatory-raptor-camera-server.spec
	${RPMBUILD} -ba observatory-raptor-camera-client.spec
	${RPMBUILD} -ba python3-warwick-observatory-raptor-camera.spec
	${RPMBUILD} -ba clasp-raptor-camera-data.spec
	mv build/noarch/*.rpm .
	rm -rf build
	mv raptor_camd.bak raptor_camd


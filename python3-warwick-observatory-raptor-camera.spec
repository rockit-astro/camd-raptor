Name:      python3-warwick-observatory-raptor-camera
Version:   20230617
Release:   0
License:   GPL3
Summary:   Common code for the Raptor camera daemon
Url:       https://github.com/warwick-one-metre/raptor-camd
BuildArch: noarch
Requires:  python3-numpy python3-astropy python3-warwick-observatory-common

%description

%prep

rsync -av --exclude=build .. .

%build
%{__python3} setup.py build

%install
%{__python3} setup.py install --prefix=%{_prefix} --root=%{buildroot}

%files
%defattr(-,root,root,-)
%{python3_sitelib}/*

%changelog

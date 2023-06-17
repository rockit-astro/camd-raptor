Name:      observatory-raptor-camera-server
Version:   20230617
Release:   0
Url:       https://github.com/warwick-one-metre/raptor-camd
Summary:   Control server for Raptor Ninox 1280 SWIR camera
License:   GPL-3.0
Group:     Unspecified
BuildArch: noarch
Requires:  python3 python3-Pyro4 python3-numpy python3-astropy python3-warwick-observatory-common python3-warwick-observatory-raptor-camera

%description

%build
mkdir -p %{buildroot}%{_bindir}
mkdir -p %{buildroot}%{_unitdir}
mkdir -p %{buildroot}%{_udevrulesdir}

%{__install} %{_sourcedir}/raptor_camd %{buildroot}%{_bindir}
%{__install} %{_sourcedir}/raptor_camd@.service %{buildroot}%{_unitdir}

%files
%defattr(0755,root,root,-)
%{_bindir}/raptor_camd
%defattr(0644,root,root,-)

%{_unitdir}/raptor_camd@.service

%changelog

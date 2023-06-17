Name:      observatory-raptor-camera-client
Version:   20230617
Release:   0
Url:       https://github.com/warwick-one-metre/raptor-camd
Summary:   Control client for Raptor Ninox 1280 SWIR camera
License:   GPL-3.0
Group:     Unspecified
BuildArch: noarch
Requires:  python3 python3-Pyro4 python3-warwick-observatory-common python3-warwick-observatory-raptor-camera

%description

%build
mkdir -p %{buildroot}%{_bindir}
mkdir -p %{buildroot}/etc/bash_completion.d
%{__install} %{_sourcedir}/swir %{buildroot}%{_bindir}
%{__install} %{_sourcedir}/completion/swir %{buildroot}/etc/bash_completion.d/swir

%files
%defattr(0755,root,root,-)
%{_bindir}/swir
/etc/bash_completion.d/swir

%changelog

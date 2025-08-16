Name:      rockit-camera-raptor
Version:   %{_version}
Release:   1%{dist}
Summary:   Control code for Raptor Ninox 1280 SWIR camera
Url:       https://github.com/rockit-astro/camd-raptor
License:   GPL-3.0
BuildArch: noarch

%description


%build
mkdir -p %{buildroot}%{_bindir}
mkdir -p %{buildroot}%{_unitdir}
mkdir -p %{buildroot}%{_sysconfdir}/camd
mkdir -p %{buildroot}%{_udevrulesdir}

%{__install} %{_sourcedir}/raptor_camd %{buildroot}%{_bindir}
%{__install} %{_sourcedir}/raptor_camd@.service %{buildroot}%{_unitdir}

%{__install} %{_sourcedir}/swir.json %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/ninox1280.fmt %{buildroot}%{_sysconfdir}/camd

%package server
Summary:  Raptor camera server
Group:    Unspecified
Requires: python3-rockit-camera-raptor
%description server

%files server
%defattr(0755,root,root,-)
%{_bindir}/raptor_camd
%defattr(0644,root,root,-)
%{_unitdir}/raptor_camd@.service

%package data-clasp
Summary: Raptor camera data for the CLASP telescope
Group:   Unspecified
%description data-clasp

%files data-clasp
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/swir.json
%{_sysconfdir}/camd/ninox1280.fmt

%changelog

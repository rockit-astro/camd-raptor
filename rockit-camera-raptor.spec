Name:      rockit-camera-raptor
Version:   %{_version}
Release:   1
Summary:   Control code for Raptor Ninox 1280 SWIR camera
Url:       https://github.com/rockit-astro/raptor-camd
License:   GPL-3.0
BuildArch: noarch

%description


%build
mkdir -p %{buildroot}%{_bindir}
mkdir -p %{buildroot}%{_unitdir}
mkdir -p %{buildroot}%{_sysconfdir}/camd
mkdir -p %{buildroot}%{_sysconfdir}/bash_completion.d
mkdir -p %{buildroot}%{_udevrulesdir}

%{__install} %{_sourcedir}/raptor_camd %{buildroot}%{_bindir}
%{__install} %{_sourcedir}/raptor_camd@.service %{buildroot}%{_unitdir}
%{__install} %{_sourcedir}/swir %{buildroot}%{_bindir}
%{__install} %{_sourcedir}/completion/swir %{buildroot}%{_sysconfdir}/bash_completion.d/swir

%{__install} %{_sourcedir}/cam2.json %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/cam2.fmt %{buildroot}%{_sysconfdir}/camd

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

%package client
Summary:  Raptor camera client
Group:    Unspecified
Requires: python3-rockit-camera-raptor
%description client

%files client
%defattr(0755,root,root,-)
%{_bindir}/swir
%{_sysconfdir}/bash_completion.d/swir

%package data-clasp
Summary: Raptor camera data for the CLASP telescope
Group:   Unspecified
%description data-clasp

%files data-clasp
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/cam2.json
%{_sysconfdir}/camd/cam2.fmt

%changelog

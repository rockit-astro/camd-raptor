Name:      clasp-raptor-camera-data
Version:   20230617
Release:   0
Url:       https://github.com/warwick-one-metre/raptor-camd
Summary:   Camera configuration for the CLASP telescope.
License:   GPL-3.0
Group:     Unspecified
BuildArch: noarch

%description

%build
mkdir -p %{buildroot}%{_sysconfdir}/camd
%{__install} %{_sourcedir}/cam2.json %{buildroot}%{_sysconfdir}/camd

%files
%defattr(0644,root,root,-)
%{_sysconfdir}/camd/cam2.json

%changelog

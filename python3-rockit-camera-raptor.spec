Name:           python3-rockit-camera-raptor
Version:        %{_version}
Release:        1%{dist}
License:        GPL3
Summary:        Common code for the Raptor camera daemon
Url:            https://github.com/rockit-astro/camd-raptor
BuildArch:      noarch
BuildRequires:  python3-devel

%description

%prep
rsync -av --exclude=build --exclude=.git --exclude=.github .. .

%generate_buildrequires
%pyproject_buildrequires -R

%build
%pyproject_wheel

%install
%pyproject_install
%pyproject_save_files rockit

%files -f %{pyproject_files}

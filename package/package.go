package rsensors

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	cunittest "github.com/jurgen-kluft/cunittest/package"
	rcore "github.com/jurgen-kluft/rcore/package"
)

const (
	repo_path = "github.com\\jurgen-kluft"
	repo_name = "rsensors"
)

// rsensors is a  package for Arduino projects that holds a
// couple of sensor objects, namely:
// - BME280 (temperature, humidity, pressure)
// - BH1750 (light sensor)
// - Sensirion/SCD41 (carbon dioxide sensor, temperature, humidity)
func GetPackage() *denv.Package {
	name := repo_name

	// dependencies
	cunittestpkg := cunittest.GetPackage()
	corepkg := rcore.GetPackage()

	// main package
	mainpkg := denv.NewPackage(repo_path, repo_name)
	mainpkg.AddPackage(corepkg)
	mainpkg.AddPackage(cunittestpkg)

	// TODO, make a library per sensor ?

	// main library
	mainLib := denv.SetupCppLibProject(mainpkg, name)
	mainLib.AddDependencies(corepkg.GetMainLib())

	// test library
	mainTestLib := denv.SetupCppTestLibProject(mainpkg, name)
	mainTestLib.AddDependencies(corepkg.GetTestLib())

	// BH1750 library
	bh1750Lib := denv.SetupCppLibraryForArduinoEsp32(mainpkg, "lib_bh1750", "bh1750")
	bh1750Lib.AddDependencies(corepkg.GetMainLib())

	// BME280 library
	bme280Lib := denv.SetupCppLibraryForArduinoEsp32(mainpkg, "lib_bme280", "bme280")
	bme280Lib.AddDependencies(corepkg.GetMainLib())

	// SCD41 library
	scd41Lib := denv.SetupCppLibraryForArduinoEsp32(mainpkg, "lib_scd41", "scd41")
	scd41Lib.AddDependencies(corepkg.GetMainLib())

	// RD03D library
	rd03dLib := denv.SetupCppLibraryForArduinoEsp32(mainpkg, "lib_rd03d", "rd03d")
	rd03dLib.AddDependencies(corepkg.GetMainLib())

	// HSP24 library
	hsp24Lib := denv.SetupCppLibraryForArduinoEsp32(mainpkg, "lib_hsp24", "hsp24")
	hsp24Lib.AddDependencies(corepkg.GetMainLib())
	hsp24Lib.AddDependency(mainLib)

	// HMMD library
	hmmdLib := denv.SetupCppLibraryForArduinoEsp32(mainpkg, "lib_hmmd", "hmmd")
	hmmdLib.AddDependencies(corepkg.GetMainLib())
	hmmdTestLib := denv.SetupCppTestLibProject(mainpkg, "hmmd")
	hmmdTestLib.AddDependencies(corepkg.GetTestLib())
	hmmdTestLib.AddDependency(mainTestLib)

	// mg58f18 library
	mg58f18Lib := denv.SetupCppLibraryForArduinoEsp32(mainpkg, "lib_mg58f18", "mg58f18")
	mg58f18Lib.AddDependencies(corepkg.GetMainLib())

	// unittest project
	maintest := denv.SetupCppTestProject(mainpkg, name)
	maintest.AddDependencies(cunittestpkg.GetMainLib())
	maintest.AddDependency(mainTestLib)

	mainpkg.AddMainLib(mainLib)
	mainpkg.AddLibrary(bh1750Lib)
	mainpkg.AddLibrary(bme280Lib)
	mainpkg.AddLibrary(scd41Lib)
	mainpkg.AddLibrary(rd03dLib)
	mainpkg.AddLibrary(hsp24Lib)
	mainpkg.AddLibrary(hmmdLib)
	mainpkg.AddLibrary(mg58f18Lib)

	mainpkg.AddTestLib(mainTestLib)
	mainpkg.AddUnittest(maintest)

	return mainpkg
}

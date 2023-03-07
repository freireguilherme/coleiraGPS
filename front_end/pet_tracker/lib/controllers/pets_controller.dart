import 'package:flutter/material.dart';
import 'package:geolocator/geolocator.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';
import 'package:pet_tracker/pages/pet_tracker.dart';
import 'package:pet_tracker/repositories/pets_repository.dart';
import 'package:pet_tracker/widgets/pet_details.dart';

class PetsController extends ChangeNotifier {
  double lat = 0.0;
  double long = 0.0;
  String erro = '';
  late GoogleMapController _mapsController;
  Set<Marker> markers = Set<Marker>();

  get mapsController => _mapsController;

  onMapCreated(GoogleMapController gmc) async {
    _mapsController = gmc;
    getPosition();
    loadPets();
  }

  loadPets() {
    final pets = PetsRepository().pets;
    pets.forEach((pet) async {
      markers.add(
        Marker(
          markerId: MarkerId(pet.name),
          position: LatLng(pet.lat, pet.long),
          icon: await BitmapDescriptor.fromAssetImage(
              const ImageConfiguration(size: Size(5, 5)), 'images/dog.png'),
          onTap: () => {
            showModalBottomSheet(
              context: appKey.currentState!.context,
              builder: (context) => PetDetails(pet: pet),
            )
          },
        ),
      );
    });
    notifyListeners();
  }

  getPosition() async {
    try {
      Position position = await _currentPosition();
      lat = position.latitude;
      long = position.longitude;
      _mapsController.animateCamera(CameraUpdate.newLatLng(LatLng(lat, long)));
    } catch (e) {
      erro = e.toString();
    }
    notifyListeners();
  }

  Future<Position> _currentPosition() async {
    LocationPermission permission;
    bool active = await Geolocator.isLocationServiceEnabled();
    if (!active) {
      return Future.error('Por favor, habilite a localizaçã no smartphone');
    }

    permission = await Geolocator.checkPermission();
    if (permission == LocationPermission.denied) {
      permission = await Geolocator.requestPermission();
      if (permission == LocationPermission.denied) {
        return Future.error('Você precisa autorizar o acesso à localização');
      }
    }

    if (permission == LocationPermission.deniedForever) {
      return Future.error('Você precisa autorizar o acesso à localização');
    }

    return await Geolocator.getCurrentPosition();
  }
}

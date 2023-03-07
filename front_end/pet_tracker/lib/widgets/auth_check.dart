import 'package:flutter/material.dart';
import 'package:pet_tracker/pages/pet_tracker.dart';
import 'package:pet_tracker/pages/login_page.dart';
import 'package:pet_tracker/services/auth_service.dart';
import 'package:provider/provider.dart';

class AuthCheck extends StatefulWidget {
  const AuthCheck({super.key});

  @override
  State<AuthCheck> createState() => _AuthCheckState();
}

class _AuthCheckState extends State<AuthCheck> {
  @override
  Widget build(BuildContext context) {
    AuthService auth = Provider.of<AuthService>(context);
    if (auth.isLoading)
      return loading();
    else if (auth.user == null)
      return LoginPage();
    else
      return PetTrackerPage();
  }

  loading() {
    return Scaffold(
      body: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Image.asset('images/dog.png'),
          CircularProgressIndicator(),
        ],
      ),
    );
  }
}

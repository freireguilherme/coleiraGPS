import 'package:flutter/material.dart';
import 'package:flutter_config/flutter_config.dart';
import 'package:pet_tracker/widgets/auth_check.dart';
import 'package:pet_tracker/repositories/pets_repository.dart';
import 'package:pet_tracker/services/auth_service.dart';
import 'package:provider/provider.dart';
import 'package:firebase_core/firebase_core.dart';
import 'firebase_options.dart';


void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await FlutterConfig.loadEnvVariables();
  await Firebase.initializeApp(
    options: DefaultFirebaseOptions.currentPlatform,
  );

  runApp(MultiProvider(
    providers: [
      ChangeNotifierProvider(create: (context) => AuthService()),
      ChangeNotifierProvider<PetsRepository>(
        create: (_) => PetsRepository(),
      )
    ],
    child: App(),
  ));
}

class App extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Pet Tracking',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        primarySwatch: Colors.indigo,
      ),
      home: AuthCheck(),
    );
  }
}
